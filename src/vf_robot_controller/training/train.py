"""
train.py — supervised training loop for MetaCriticMLP.

Usage:
  cd ~/cogni-nav-x0/src/vf_robot_controller
  python3 training/train.py

What it does:
  1. Loads all HDF5 files from training/data/
  2. Generates labels via hindsight trajectory ranking (dataset.py)
  3. Trains MetaCriticMLP with CrossEntropyLoss
  4. Saves best checkpoint to meta_critic/models/meta_critic.pt (TorchScript)
  5. Plots training/validation loss curve to training/figures/loss_curve.png
"""

import os
import sys
import time
import numpy as np
import matplotlib.pyplot as plt

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader, random_split

# Allow running from package root
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from training.model   import build_model
from training.dataset import MetaCriticDataset


# ── Hyperparameters ────────────────────────────────────────────────────────────
DATA_DIR    = 'training/data'
OUTPUT_DIR  = 'meta_critic/models'
FIGURES_DIR = 'training/figures'
EPOCHS      = 100
BATCH_SIZE  = 256
LR          = 1e-3
WEIGHT_DECAY = 1e-4
VAL_SPLIT   = 0.2
PATIENCE    = 10       # early stopping patience (epochs)
NUM_CRITICS = 10


def train(
    data_dir:   str = DATA_DIR,
    output_dir: str = OUTPUT_DIR,
    epochs:     int = EPOCHS,
    batch_size: int = BATCH_SIZE,
    lr:         float = LR,
):
    os.makedirs(output_dir,  exist_ok=True)
    os.makedirs(FIGURES_DIR, exist_ok=True)

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f'Device: {device}')

    # ── Dataset ───────────────────────────────────────────────────────────────
    print('Loading dataset...')
    dataset = MetaCriticDataset(data_dir=data_dir, n_critics=NUM_CRITICS)

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

    print(f'Train: {n_train:,}  Val: {n_val:,}')

    # ── Model ─────────────────────────────────────────────────────────────────
    model = build_model(input_dim=410, output_dim=NUM_CRITICS).to(device)
    print(f'Parameters: {model.param_count():,}')

    # ── Optimiser + scheduler ─────────────────────────────────────────────────
    optimiser = torch.optim.Adam(
        model.parameters(), lr=lr, weight_decay=WEIGHT_DECAY)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimiser, T_max=epochs, eta_min=lr * 0.01)

    # ── Training loop ─────────────────────────────────────────────────────────
    best_val_loss = float('inf')
    patience_count = 0
    train_losses, val_losses = [], []

    print(f'\nTraining for up to {epochs} epochs (early stop patience={PATIENCE})...\n')

    for epoch in range(1, epochs + 1):
        t0 = time.time()

        # Train
        model.train()
        train_loss = 0.0
        for features, labels in train_loader:
            features = features.to(device)
            labels   = labels.to(device)

            optimiser.zero_grad()
            weights = model(features)          # (B, 10) softmax

            # CrossEntropyLoss expects logits — use pre-softmax logits
            # Re-run through net without softmax for loss computation
            logits = model.net(features)       # (B, 10) raw
            loss   = F.cross_entropy(logits, labels)

            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimiser.step()
            train_loss += loss.item() * len(features)

        train_loss /= n_train

        # Validate
        model.eval()
        val_loss = 0.0
        correct  = 0
        with torch.no_grad():
            for features, labels in val_loader:
                features = features.to(device)
                labels   = labels.to(device)
                logits   = model.net(features)
                val_loss += F.cross_entropy(logits, labels).item() * len(features)
                correct  += (logits.argmax(dim=-1) == labels).sum().item()

        val_loss /= n_val
        val_acc   = correct / n_val * 100.0

        scheduler.step()
        train_losses.append(train_loss)
        val_losses.append(val_loss)

        elapsed = time.time() - t0
        print(f'Epoch {epoch:3d}/{epochs}  '
              f'train={train_loss:.4f}  val={val_loss:.4f}  '
              f'acc={val_acc:.1f}%  lr={scheduler.get_last_lr()[0]:.2e}  '
              f't={elapsed:.1f}s')

        # Checkpoint best model
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            patience_count = 0
            _save_model(model, output_dir)
            print(f'  -> New best  val_loss={best_val_loss:.4f}  (saved)')
        else:
            patience_count += 1
            if patience_count >= PATIENCE:
                print(f'\nEarly stopping at epoch {epoch} '
                      f'(no improvement for {PATIENCE} epochs)')
                break

    # ── Save loss curve ────────────────────────────────────────────────────────
    _plot_loss(train_losses, val_losses, FIGURES_DIR)
    print(f'\nTraining complete. Best val_loss={best_val_loss:.4f}')
    print(f'Model saved to {output_dir}/meta_critic.pt')


def _save_model(model: nn.Module, output_dir: str):
    """Export to TorchScript — loadable by both Python and C++ libtorch."""
    model.eval()
    scripted = torch.jit.script(model)
    path = os.path.join(output_dir, 'meta_critic.pt')
    scripted.save(path)


def _plot_loss(train_losses, val_losses, figures_dir):
    fig, ax = plt.subplots(figsize=(8, 4))
    epochs = range(1, len(train_losses) + 1)
    ax.plot(epochs, train_losses, label='train')
    ax.plot(epochs, val_losses,   label='val')
    ax.set_xlabel('Epoch')
    ax.set_ylabel('CrossEntropyLoss')
    ax.set_title('Meta-critic supervised training')
    ax.legend()
    ax.grid(True, alpha=0.3)
    path = os.path.join(figures_dir, 'loss_curve.png')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Loss curve saved to {path}')


if __name__ == '__main__':
    train()
