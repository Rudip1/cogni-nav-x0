"""
evaluate.py — model evaluation and thesis figures.

Usage:
  cd ~/cogni-nav-x0/src/vf_robot_controller
  python3 training/evaluate.py

Generates:
  training/figures/weights_vs_corridor.png  ← KEY THESIS FIGURE
    Shows how each critic's learned weight changes as corridor width varies.
    Expected: separation critics (obstacle, clearance) increase in narrow corridors.
              goal/velocity critics decrease in narrow corridors.

  training/figures/weights_vs_clutter.png
    Same but varying clutter density scalar.

  training/figures/weights_heatmap.png
    2D heatmap: corridor width × clutter density → each critic weight.
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

import torch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from training.model import build_model


FIGURES_DIR  = 'training/figures'
MODEL_PATH   = 'meta_critic/models/meta_critic.pt'
FEATURE_DIM  = 410
NUM_CRITICS  = 10

# Slot → name mapping (must match critics.xml order)
CRITIC_NAMES = [
    'obstacle',       # 0  separation
    'volumetric',     # 1  separation
    'dynamic_obs',    # 2  separation
    'path_follow',    # 3  alignment
    'smoothness',     # 4  alignment
    'goal',           # 5  goal-seeking
    'velocity',       # 6  goal-seeking
    'corridor',       # 7  cohesion
    'clearance',      # 8  separation
    'oscillation',    # 9  alignment
]

REYNOLDS_GROUPS = {
    'separation':  [0, 1, 2, 8],
    'alignment':   [3, 4, 9],
    'cohesion':    [7],
    'goal':        [5, 6],
}

GROUP_COLORS = {
    'separation': '#D85A30',   # coral
    'alignment':  '#378ADD',   # blue
    'cohesion':   '#1D9E75',   # teal
    'goal':       '#7F77DD',   # purple
}


def load_model(model_path: str):
    """Load TorchScript model. Falls back to untrained model if not found."""
    try:
        model = torch.jit.load(model_path, map_location='cpu')
        model.eval()
        print(f'Loaded trained model from {model_path}')
        return model, True
    except Exception as e:
        print(f'Warning: could not load {model_path}: {e}')
        print('Generating plots with UNTRAINED model (for structure verification only)')
        model = build_model(input_dim=FEATURE_DIM, output_dim=NUM_CRITICS)
        model.eval()
        return model, False


def make_feature_vector(
    corridor_w: float = 2.0,
    gcf_mean:   float = 0.5,
    gcf_max:    float = 0.5,
    num_dyn:    float = 0.0,
    closest:    float = 2.0,
    vx:         float = 0.3,
    vz:         float = 0.0,
    goal_dist:  float = 5.0,
    goal_head:  float = 0.0,
    path_dev:   float = 0.0,
) -> np.ndarray:
    """Build a synthetic 410-dim feature vector with given scalar values."""
    # Costmap patch: neutral (0.1 = free space)
    patch = np.full(400, 0.1, dtype=np.float32)

    scalars = np.array([
        np.clip(corridor_w, 0.0, 5.0) / 5.0,
        np.clip(gcf_mean,   0.0, 1.0),
        np.clip(gcf_max,    0.0, 1.0),
        np.clip(num_dyn,    0.0, 10.0) / 10.0,
        np.clip(closest,    0.0, 5.0)  / 5.0,
        np.clip(vx / 1.0,  -1.0, 1.0),
        np.clip(vz / 1.0,  -1.0, 1.0),
        np.clip(goal_dist,  0.0, 20.0) / 20.0,
        np.clip(goal_head, -np.pi, np.pi) / np.pi,
        np.clip(path_dev,   0.0, 2.0)  / 2.0,
    ], dtype=np.float32)

    return np.concatenate([patch, scalars])


@torch.no_grad()
def get_weights(model, feature_vec: np.ndarray) -> np.ndarray:
    x = torch.from_numpy(feature_vec).unsqueeze(0)
    return model(x).squeeze(0).numpy()


def plot_weights_vs_corridor(model, trained: bool, out_dir: str):
    """Key thesis figure: critic weights as corridor width varies 0.5m → 4m."""
    widths = np.linspace(0.5, 4.0, 80)
    all_weights = []

    for w in widths:
        # As corridor narrows, GCF increases
        gcf = np.clip(1.0 - (w - 0.5) / 3.5, 0.1, 0.95)
        feat = make_feature_vector(corridor_w=w, gcf_mean=gcf, gcf_max=gcf)
        all_weights.append(get_weights(model, feat))

    all_weights = np.array(all_weights)   # (80, 10)

    fig, ax = plt.subplots(figsize=(10, 5))

    for group, slots in REYNOLDS_GROUPS.items():
        color = GROUP_COLORS[group]
        for slot in slots:
            ax.plot(widths, all_weights[:, slot],
                    label=f'{CRITIC_NAMES[slot]} ({group})',
                    color=color, alpha=0.85,
                    linewidth=2 if slot in [0, 5] else 1.2)

    ax.axvline(x=1.2, color='gray', linestyle='--', alpha=0.5, label='min corridor (1.2m)')
    ax.axvline(x=0.9, color='red',  linestyle='--', alpha=0.5, label='doorway (0.9m)')
    ax.set_xlabel('Corridor width (m)', fontsize=12)
    ax.set_ylabel('Learned critic weight', fontsize=12)
    title = 'Meta-critic weights vs corridor width'
    if not trained:
        title += ' [UNTRAINED — for structure only]'
    ax.set_title(title, fontsize=13)
    ax.legend(loc='upper right', fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0, None)

    path = os.path.join(out_dir, 'weights_vs_corridor.png')
    fig.savefig(path, dpi=200, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_weights_vs_clutter(model, trained: bool, out_dir: str):
    """Critic weights as clutter density varies."""
    clutter_levels = np.linspace(0.0, 1.0, 60)
    all_weights = []

    for c in clutter_levels:
        feat = make_feature_vector(gcf_mean=c, gcf_max=c, closest=max(0.3, 2.0 - c * 1.5))
        all_weights.append(get_weights(model, feat))

    all_weights = np.array(all_weights)

    fig, ax = plt.subplots(figsize=(10, 5))
    for group, slots in REYNOLDS_GROUPS.items():
        color = GROUP_COLORS[group]
        for slot in slots:
            ax.plot(clutter_levels, all_weights[:, slot],
                    label=f'{CRITIC_NAMES[slot]}',
                    color=color, alpha=0.85, linewidth=1.5)

    ax.set_xlabel('Clutter density (normalised GCF)', fontsize=12)
    ax.set_ylabel('Learned critic weight', fontsize=12)
    title = 'Meta-critic weights vs clutter density'
    if not trained:
        title += ' [UNTRAINED]'
    ax.set_title(title, fontsize=13)
    ax.legend(loc='upper right', fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)

    path = os.path.join(out_dir, 'weights_vs_clutter.png')
    fig.savefig(path, dpi=200, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def plot_weights_heatmap(model, trained: bool, out_dir: str):
    """2D heatmap: corridor width × clutter → weight for each critic."""
    widths   = np.linspace(0.5, 4.0, 30)
    clutters = np.linspace(0.0, 1.0, 30)

    fig, axes = plt.subplots(2, 5, figsize=(18, 7))
    axes = axes.flatten()

    for slot in range(NUM_CRITICS):
        grid = np.zeros((len(clutters), len(widths)))
        for i, c in enumerate(clutters):
            for j, w in enumerate(widths):
                gcf = np.clip(1.0 - (w - 0.5) / 3.5, 0.1, 0.95) * 0.5 + c * 0.5
                feat = make_feature_vector(corridor_w=w, gcf_mean=gcf, gcf_max=gcf)
                grid[i, j] = get_weights(model, feat)[slot]

        im = axes[slot].imshow(
            grid, origin='lower', aspect='auto',
            extent=[widths[0], widths[-1], clutters[0], clutters[-1]],
            cmap='RdYlGn_r', vmin=0.0, vmax=0.3)
        axes[slot].set_title(f'{CRITIC_NAMES[slot]}\n(slot {slot})', fontsize=9)
        axes[slot].set_xlabel('corridor (m)', fontsize=7)
        axes[slot].set_ylabel('clutter', fontsize=7)
        fig.colorbar(im, ax=axes[slot], fraction=0.046)

    title = 'Critic weights heatmap (corridor × clutter)'
    if not trained:
        title += ' [UNTRAINED]'
    fig.suptitle(title, fontsize=13)
    plt.tight_layout()

    path = os.path.join(out_dir, 'weights_heatmap.png')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Saved: {path}')


def main():
    os.makedirs(FIGURES_DIR, exist_ok=True)
    model, trained = load_model(MODEL_PATH)

    print('\nGenerating evaluation figures...')
    plot_weights_vs_corridor(model, trained, FIGURES_DIR)
    plot_weights_vs_clutter(model,  trained, FIGURES_DIR)
    plot_weights_heatmap(model,     trained, FIGURES_DIR)
    print('\nAll figures saved to', FIGURES_DIR)


if __name__ == '__main__':
    main()
