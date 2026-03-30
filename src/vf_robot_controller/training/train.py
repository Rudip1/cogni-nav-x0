"""
train.py — supervised training for MetaCriticMLP.

─────────────────────────────────────────────────────────────────────────────
USAGE
─────────────────────────────────────────────────────────────────────────────

  # Baseline — behaviour cloning (train first, proves pipeline works)
  python3 training/train.py --method IMITATION

  # Novel contribution — ideal weight recovery (thesis claim)
  python3 training/train.py --method META_CRITIC

  # Full options
  python3 training/train.py --method META_CRITIC \
      --data-dir training/data \
      --epochs 150 \
      --batch-size 256 \
      --lr 1e-3

─────────────────────────────────────────────────────────────────────────────
WHAT CHANGES BETWEEN METHODS
─────────────────────────────────────────────────────────────────────────────

  IMITATION:
    Dataset label : int (best trajectory index)
    Loss          : CrossEntropyLoss(logits, best_idx)
    Validation    : top-1 accuracy (% correct trajectory selected)
    Output file   : meta_critic_IMITATION_best.pt

  META_CRITIC:
    Dataset label : float32[K] ideal weight vector
    Loss          : MSELoss(predicted_weights, ideal_weights)
    Validation    : cosine similarity between predicted and ideal weights
    Output file   : meta_critic_META_CRITIC_best.pt

  Everything else is identical:
    Network architecture, optimizer, scheduler, early stopping,
    TorchScript export, loss curve figure, deployment format.

─────────────────────────────────────────────────────────────────────────────
OUTPUT FILES (stamped with method name — no accidental overwrites)
─────────────────────────────────────────────────────────────────────────────

  training/figures/loss_curve_IMITATION.png
  training/figures/loss_curve_META_CRITIC.png
  training/checkpoints/meta_critic_IMITATION_best.pt    ← TorchScript
  training/checkpoints/meta_critic_META_CRITIC_best.pt  ← TorchScript

  To deploy after training:
    cp training/checkpoints/meta_critic_META_CRITIC_best.pt \\
       meta_critic/models/meta_critic.pt
"""

import os
import sys
import time
import argparse
import numpy as np
import matplotlib.pyplot as plt

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader, random_split

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from training.dataset import MetaCriticDataset, IMITATION, META_CRITIC, VALID_METHODS
from training.model   import build_model


# ── Defaults ──────────────────────────────────────────────────────────────────
DATA_DIR      = 'training/data'
FIGURES_DIR   = 'training/figures'
CKPT_DIR      = 'training/checkpoints'
EPOCHS        = 100
BATCH_SIZE    = 256
LR            = 1e-3
WEIGHT_DECAY  = 1e-4
VAL_SPLIT     = 0.2
PATIENCE      = 10
NUM_CRITICS   = 10


# ── Loss functions ────────────────────────────────────────────────────────────

def compute_loss(method: str, model, features, labels, device):
    """
    Unified loss computation — branches on method.

    IMITATION:
      Uses raw logits (pre-softmax) with CrossEntropyLoss.
      labels: int64 (N,)

    META_CRITIC:
      Uses softmax output with MSELoss against ideal weight vector.
      labels: float32 (N, K)
    """
    if method == IMITATION:
        logits = model.net(features)                  # (B, K) raw logits
        loss   = F.cross_entropy(logits, labels)
        return loss, logits

    else:  # META_CRITIC
        weights = model(features)                     # (B, K) softmax output
        loss    = F.mse_loss(weights, labels.float())
        return loss, weights


def compute_val_metric(method: str, outputs, labels):
    """
    Validation metric — branches on method.

    IMITATION:   top-1 accuracy (fraction of correct trajectory selections)
    META_CRITIC: mean cosine similarity between predicted and ideal weights
    """
    if method == IMITATION:
        correct = (outputs.argmax(dim=-1) == labels).sum().item()
        return correct, 'accuracy'

    else:  # META_CRITIC
        # Cosine similarity: 1.0 = perfect, 0.0 = orthogonal
        cos_sim = F.cosine_similarity(outputs, labels.float(), dim=-1)
        return cos_sim.sum().item(), 'cosine_sim'


# ── Main training loop ────────────────────────────────────────────────────────

def train(
    method:     str   = IMITATION,
    data_dir:   str   = DATA_DIR,
    epochs:     int   = EPOCHS,
    batch_size: int   = BATCH_SIZE,
    lr:         float = LR,
):
    os.makedirs(FIGURES_DIR, exist_ok=True)
    os.makedirs(CKPT_DIR,    exist_ok=True)

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f'[{method}] Device: {device}')

    # ── Dataset ───────────────────────────────────────────────────────────────
    print(f'[{method}] Loading dataset from {data_dir}...')
    dataset = MetaCriticDataset(
        data_dir=data_dir, method=method, n_critics=NUM_CRITICS)

    n_val   = max(1, int(len(dataset) * VAL_SPLIT))
    n_train = len(dataset) - n_val
    train_set, val_set = random_split(
        dataset, [n_train, n_val],
        generator=torch.Generator().manual_seed(42))

    train_loader = DataLoader(
        train_set, batch_size=batch_size, shuffle=True,
        num_workers=2, pin_memory=(device.type == 'cuda'))
    val_loader = DataLoader(
        val_set, batch_size=batch_size, shuffle=False,
        num_workers=2, pin_memory=(device.type == 'cuda'))

    print(f'[{method}] Train: {n_train:,}  Val: {n_val:,}')

    # ── Model ─────────────────────────────────────────────────────────────────
    model = build_model(input_dim=410, output_dim=NUM_CRITICS).to(device)
    print(f'[{method}] Parameters: {model.param_count():,}')

    # ── Optimiser + scheduler ─────────────────────────────────────────────────
    optimiser = torch.optim.Adam(
        model.parameters(), lr=lr, weight_decay=WEIGHT_DECAY)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimiser, T_max=epochs, eta_min=lr * 0.01)

    # ── Training loop ─────────────────────────────────────────────────────────
    best_val_loss  = float('inf')
    patience_count = 0
    train_losses, val_losses = [], []
    metric_name    = 'accuracy' if method == IMITATION else 'cosine_sim'

    print(f'\n[{method}] Training up to {epochs} epochs '
          f'(early stop patience={PATIENCE})...\n')

    for epoch in range(1, epochs + 1):
        t0 = time.time()

        # ── Train ─────────────────────────────────────────────────────────────
        model.train()
        train_loss = 0.0
        for features, labels in train_loader:
            features = features.to(device)
            labels   = labels.to(device)

            optimiser.zero_grad()
            loss, _ = compute_loss(method, model, features, labels, device)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimiser.step()
            train_loss += loss.item() * len(features)

        train_loss /= n_train

        # ── Validate ──────────────────────────────────────────────────────────
        model.eval()
        val_loss   = 0.0
        metric_sum = 0.0
        with torch.no_grad():
            for features, labels in val_loader:
                features = features.to(device)
                labels   = labels.to(device)
                loss, outputs = compute_loss(
                    method, model, features, labels, device)
                val_loss   += loss.item() * len(features)
                m, _        = compute_val_metric(method, outputs, labels)
                metric_sum += m

        val_loss    /= n_val
        metric_val   = metric_sum / n_val

        scheduler.step()
        train_losses.append(train_loss)
        val_losses.append(val_loss)

        print(f'[{method}] Epoch {epoch:3d}/{epochs}  '
              f'train={train_loss:.4f}  val={val_loss:.4f}  '
              f'{metric_name}={metric_val:.3f}  '
              f'lr={scheduler.get_last_lr()[0]:.2e}  '
              f't={time.time()-t0:.1f}s')

        # ── Checkpoint ────────────────────────────────────────────────────────
        if val_loss < best_val_loss:
            best_val_loss  = val_loss
            patience_count = 0
            ckpt_path = os.path.join(
                CKPT_DIR, f'meta_critic_{method}_best.pt')
            _save_torchscript(model, ckpt_path)
            print(f'[{method}]   -> Best val_loss={best_val_loss:.4f}  saved to {ckpt_path}')
        else:
            patience_count += 1
            if patience_count >= PATIENCE:
                print(f'\n[{method}] Early stopping at epoch {epoch}')
                break

    # ── Save loss curve ────────────────────────────────────────────────────────
    fig_path = os.path.join(FIGURES_DIR, f'loss_curve_{method}.png')
    _plot_loss(train_losses, val_losses, method, fig_path)

    print(f'\n[{method}] Training complete.')
    print(f'[{method}] Best val_loss: {best_val_loss:.4f}')
    print(f'[{method}] Checkpoint:   {CKPT_DIR}/meta_critic_{method}_best.pt')
    print(f'[{method}] Loss curve:   {fig_path}')
    print(f'\nTo deploy this model:')
    print(f'  cp {CKPT_DIR}/meta_critic_{method}_best.pt meta_critic/models/meta_critic.pt')
    print(f'Then launch: ros2 launch vf_robot_controller inference_launch.py')


# ── Helpers ───────────────────────────────────────────────────────────────────

def _save_torchscript(model: nn.Module, path: str):
    """Export to TorchScript — loadable by inference_node.py and C++ libtorch."""
    model.eval()
    scripted = torch.jit.script(model)
    scripted.save(path)


def _plot_loss(train_losses, val_losses, method: str, path: str):
    fig, ax = plt.subplots(figsize=(8, 4))
    epochs  = range(1, len(train_losses) + 1)
    ax.plot(epochs, train_losses, label='train')
    ax.plot(epochs, val_losses,   label='val')
    ax.set_xlabel('Epoch')
    loss_label = 'CrossEntropyLoss' if method == IMITATION else 'MSELoss'
    ax.set_ylabel(loss_label)
    ax.set_title(f'Meta-critic training — {method}')
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'[{method}] Loss curve saved to {path}')


# ── CLI ───────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Train MetaCriticMLP',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--method', required=True, choices=VALID_METHODS,
        help='IMITATION = behaviour cloning baseline | '
             'META_CRITIC = novel weight recovery (thesis contribution)')
    parser.add_argument('--data-dir',   default=DATA_DIR)
    parser.add_argument('--epochs',     type=int,   default=EPOCHS)
    parser.add_argument('--batch-size', type=int,   default=BATCH_SIZE)
    parser.add_argument('--lr',         type=float, default=LR)
    args = parser.parse_args()

    train(
        method     = args.method,
        data_dir   = args.data_dir,
        epochs     = args.epochs,
        batch_size = args.batch_size,
        lr         = args.lr,
    )
