"""
train.py — supervised training for MetaCriticMLP and ImitationMLP.

─────────────────────────────────────────────────────────────────────────────
USAGE
─────────────────────────────────────────────────────────────────────────────

  # META_CRITIC (novel contribution — ideal weight recovery)
  python3 training/train.py --method META_CRITIC --data-dir training/data --epochs 100

  # IMITATION (behaviour cloning baseline)
  python3 training/train.py --method IMITATION --data-dir training/imitation --epochs 100

─────────────────────────────────────────────────────────────────────────────
META_CRITIC
─────────────────────────────────────────────────────────────────────────────
  Dataset label : float32[K] ideal weight vector
  Loss          : MSELoss(predicted_weights, ideal_weights)
  Validation    : cosine similarity between predicted and ideal weights
  Output file   : training/checkpoints/meta_critic_best.pt

─────────────────────────────────────────────────────────────────────────────
IMITATION
─────────────────────────────────────────────────────────────────────────────
  Dataset label : float32[2]  [linear_x, angular_z] from teacher /cmd_vel
  Loss          : MSELoss(predicted_vel, teacher_vel)
  Output file   : training/checkpoints/imitation_best.pt
─────────────────────────────────────────────────────────────────────────────
"""

import os
import sys
import time
import argparse
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader, random_split

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from training.dataset import MetaCriticDataset, META_CRITIC
from training.model import build_model


# ── Defaults ──────────────────────────────────────────────────────────────────
DATA_DIR = "training/data"
FIGURES_DIR = "training/figures"
CKPT_DIR = "training/checkpoints"
EPOCHS = 100
BATCH_SIZE = 256
LR = 1e-3
WEIGHT_DECAY = 1e-4
VAL_SPLIT = 0.2
PATIENCE = 15
NUM_CRITICS = 10


# ── Helpers ───────────────────────────────────────────────────────────────────


def _save_torchscript(model: nn.Module, path: str):
    model.eval()
    scripted = torch.jit.script(model)
    scripted.save(path)


def _plot_loss(train_losses, val_losses, val_cosines, path: str):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))
    epochs = range(1, len(train_losses) + 1)

    ax1.plot(epochs, train_losses, label="train")
    ax1.plot(epochs, val_losses, label="val")
    ax1.set_xlabel("Epoch")
    ax1.set_ylabel("MSE Loss")
    ax1.set_title("Meta-critic training — Loss")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    ax2.plot(epochs, val_cosines, label="val cosine_sim", color="green")
    ax2.set_xlabel("Epoch")
    ax2.set_ylabel("Cosine Similarity")
    ax2.set_title("Meta-critic training — Validation Metric")
    ax2.axhline(y=1.0, color="gray", linestyle="--", alpha=0.5, label="perfect")
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim(0, 1.1)

    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[META_CRITIC] Loss curve saved to {path}")


def _plot_loss_simple(train_losses, val_losses, path: str, title: str):
    fig, ax = plt.subplots(figsize=(8, 4))
    ep = range(1, len(train_losses) + 1)
    ax.plot(ep, train_losses, label="train")
    ax.plot(ep, val_losses, label="val")
    ax.set_xlabel("Epoch")
    ax.set_ylabel("MSE Loss")
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


# ── META_CRITIC loss functions ─────────────────────────────────────────────────


def compute_loss(model, features, labels):
    weights = model(features)
    loss = F.mse_loss(weights, labels.float())
    return loss, weights


def compute_val_metric(outputs, labels):
    cos_sim = F.cosine_similarity(outputs, labels.float(), dim=-1)
    return cos_sim.sum().item()


# ── META_CRITIC training loop ─────────────────────────────────────────────────


def train(
    data_dir: str = DATA_DIR,
    epochs: int = EPOCHS,
    batch_size: int = BATCH_SIZE,
    lr: float = LR,
    n_critics: int = NUM_CRITICS,
):
    os.makedirs(FIGURES_DIR, exist_ok=True)
    os.makedirs(CKPT_DIR, exist_ok=True)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[META_CRITIC] Device: {device}")

    print(f"[META_CRITIC] Loading dataset from {data_dir}...")
    dataset = MetaCriticDataset(data_dir=data_dir, n_critics=n_critics)

    if len(dataset) == 0:
        print("[META_CRITIC] ERROR: No valid samples loaded. Check your data.")
        return

    n_val = max(1, int(len(dataset) * VAL_SPLIT))
    n_train = len(dataset) - n_val
    train_set, val_set = random_split(
        dataset, [n_train, n_val], generator=torch.Generator().manual_seed(42)
    )

    train_loader = DataLoader(
        train_set,
        batch_size=batch_size,
        shuffle=True,
        num_workers=2,
        pin_memory=(device.type == "cuda"),
    )
    val_loader = DataLoader(
        val_set,
        batch_size=batch_size,
        shuffle=False,
        num_workers=2,
        pin_memory=(device.type == "cuda"),
    )

    print(f"[META_CRITIC] Train: {n_train:,}  Val: {n_val:,}")

    model = build_model(input_dim=410, output_dim=n_critics).to(device)
    print(f"[META_CRITIC] Parameters: {model.param_count():,}")

    optimiser = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=WEIGHT_DECAY)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimiser, T_max=epochs, eta_min=lr * 0.01
    )

    best_val_loss = float("inf")
    patience_count = 0
    train_losses, val_losses, val_cosines = [], [], []

    print(
        f"\n[META_CRITIC] Training up to {epochs} epochs "
        f"(early stop patience={PATIENCE})...\n"
    )

    for epoch in range(1, epochs + 1):
        t0 = time.time()

        model.train()
        train_loss = 0.0
        for features, labels in train_loader:
            features = features.to(device)
            labels = labels.to(device)
            optimiser.zero_grad()
            loss, _ = compute_loss(model, features, labels)
            if torch.isnan(loss):
                print("[META_CRITIC] WARNING: NaN loss in training batch, skipping")
                continue
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimiser.step()
            train_loss += loss.item() * len(features)
        train_loss /= n_train

        model.eval()
        val_loss = 0.0
        cosine_sum = 0.0
        with torch.no_grad():
            for features, labels in val_loader:
                features = features.to(device)
                labels = labels.to(device)
                loss, outputs = compute_loss(model, features, labels)
                if not torch.isnan(loss):
                    val_loss += loss.item() * len(features)
                    cosine_sum += compute_val_metric(outputs, labels)
        val_loss /= n_val
        cosine_val = cosine_sum / n_val

        scheduler.step()
        train_losses.append(train_loss)
        val_losses.append(val_loss)
        val_cosines.append(cosine_val)

        print(
            f"[META_CRITIC] Epoch {epoch:3d}/{epochs}  "
            f"train={train_loss:.6f}  val={val_loss:.6f}  "
            f"cosine_sim={cosine_val:.4f}  "
            f"lr={scheduler.get_last_lr()[0]:.2e}  "
            f"t={time.time()-t0:.1f}s"
        )

        if val_loss < best_val_loss and not np.isnan(val_loss):
            best_val_loss = val_loss
            patience_count = 0
            ckpt_path = os.path.join(CKPT_DIR, "meta_critic_best.pt")
            _save_torchscript(model, ckpt_path)
            print(
                f"[META_CRITIC]   -> Best val_loss={best_val_loss:.6f}  saved to {ckpt_path}"
            )
        else:
            patience_count += 1
            if patience_count >= PATIENCE:
                print(f"\n[META_CRITIC] Early stopping at epoch {epoch}")
                break

    fig_path = os.path.join(FIGURES_DIR, "loss_curve_META_CRITIC.png")
    _plot_loss(train_losses, val_losses, val_cosines, fig_path)

    print(f"\n[META_CRITIC] Training complete.")
    print(f"[META_CRITIC] Best val_loss: {best_val_loss:.6f}")
    print(f"[META_CRITIC] Checkpoint:   {CKPT_DIR}/meta_critic_best.pt")
    print(f"[META_CRITIC] Loss curve:   {fig_path}")
    print(
        f"\nTo deploy: cp {CKPT_DIR}/meta_critic_best.pt meta_critic/models/meta_critic.pt"
    )
    print(f"Then:      ros2 launch vf_robot_controller meta_critic_inference_launch.py")


# ── IMITATION training loop ───────────────────────────────────────────────────


def train_imitation(
    data_dir: str = "training/imitation",
    epochs: int = EPOCHS,
    batch_size: int = BATCH_SIZE,
    lr: float = LR,
):
    os.makedirs(FIGURES_DIR, exist_ok=True)
    os.makedirs(CKPT_DIR, exist_ok=True)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[IMITATION] Device: {device}")

    from training.dataset import ImitationDataset
    from training.model import build_imitation_model

    print(f"[IMITATION] Loading dataset from {data_dir}...")
    dataset = ImitationDataset(data_dir=data_dir)

    if len(dataset) == 0:
        print(
            "[IMITATION] ERROR: No valid samples. Run imitation_collect_launch.py first."
        )
        return

    n_val = max(1, int(len(dataset) * VAL_SPLIT))
    n_train = len(dataset) - n_val
    train_set, val_set = random_split(
        dataset, [n_train, n_val], generator=torch.Generator().manual_seed(42)
    )

    train_loader = DataLoader(
        train_set,
        batch_size=batch_size,
        shuffle=True,
        num_workers=2,
        pin_memory=(device.type == "cuda"),
    )
    val_loader = DataLoader(
        val_set,
        batch_size=batch_size,
        shuffle=False,
        num_workers=2,
        pin_memory=(device.type == "cuda"),
    )

    print(f"[IMITATION] Train: {n_train:,}  Val: {n_val:,}")

    model = build_imitation_model(input_dim=410).to(device)
    print(f"[IMITATION] Parameters: {model.param_count():,}")

    optimiser = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=WEIGHT_DECAY)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimiser, T_max=epochs, eta_min=lr * 0.01
    )

    best_val_loss = float("inf")
    patience_count = 0
    train_losses, val_losses = [], []

    print(
        f"\n[IMITATION] Training up to {epochs} epochs "
        f"(early stop patience={PATIENCE})...\n"
    )

    for epoch in range(1, epochs + 1):
        t0 = time.time()

        model.train()
        train_loss = 0.0
        for features, labels in train_loader:
            features = features.to(device)
            labels = labels.to(device)
            optimiser.zero_grad()
            pred = model(features)
            loss = F.mse_loss(pred, labels)
            if torch.isnan(loss):
                continue
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimiser.step()
            train_loss += loss.item() * len(features)
        train_loss /= n_train

        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for features, labels in val_loader:
                features = features.to(device)
                labels = labels.to(device)
                pred = model(features)
                loss = F.mse_loss(pred, labels)
                if not torch.isnan(loss):
                    val_loss += loss.item() * len(features)
        val_loss /= n_val

        scheduler.step()
        train_losses.append(train_loss)
        val_losses.append(val_loss)

        print(
            f"[IMITATION] Epoch {epoch:3d}/{epochs}  "
            f"train={train_loss:.6f}  val={val_loss:.6f}  "
            f"lr={scheduler.get_last_lr()[0]:.2e}  "
            f"t={time.time()-t0:.1f}s"
        )

        if val_loss < best_val_loss and not np.isnan(val_loss):
            best_val_loss = val_loss
            patience_count = 0
            ckpt_path = os.path.join(CKPT_DIR, "imitation_best.pt")
            _save_torchscript(model, ckpt_path)
            print(
                f"[IMITATION]   -> Best val_loss={best_val_loss:.6f}  saved to {ckpt_path}"
            )
        else:
            patience_count += 1
            if patience_count >= PATIENCE:
                print(f"\n[IMITATION] Early stopping at epoch {epoch}")
                break

    fig_path = os.path.join(FIGURES_DIR, "loss_curve_IMITATION.png")
    _plot_loss_simple(
        train_losses,
        val_losses,
        fig_path,
        "Imitation training — velocity prediction loss",
    )

    print(f"\n[IMITATION] Training complete.")
    print(f"[IMITATION] Best val_loss: {best_val_loss:.6f}")
    print(f"[IMITATION] Checkpoint:   {CKPT_DIR}/imitation_best.pt")
    print(f"[IMITATION] Loss curve:   {fig_path}")


# ── CLI ───────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Train MetaCriticMLP or ImitationMLP",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--method", default="META_CRITIC", choices=["META_CRITIC", "IMITATION"]
    )
    parser.add_argument(
        "--data-dir",
        default=None,
        help="Data directory (default: training/data for META_CRITIC, "
        "training/imitation for IMITATION)",
    )
    parser.add_argument("--epochs", type=int, default=EPOCHS)
    parser.add_argument("--batch-size", type=int, default=BATCH_SIZE)
    parser.add_argument("--lr", type=float, default=LR)
    parser.add_argument("--n-critics", type=int, default=NUM_CRITICS)
    args = parser.parse_args()

    if args.method == "IMITATION":
        train_imitation(
            data_dir=args.data_dir or "training/imitation",
            epochs=args.epochs,
            batch_size=args.batch_size,
            lr=args.lr,
        )
    else:
        train(
            data_dir=args.data_dir or DATA_DIR,
            epochs=args.epochs,
            batch_size=args.batch_size,
            lr=args.lr,
            n_critics=args.n_critics,
        )
