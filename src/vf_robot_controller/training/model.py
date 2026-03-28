"""
model.py — MetaCriticMLP definition.

Architecture:
  Input:  410-dim feature vector
  Hidden: 256 → 128 → 64  (each with LayerNorm + ReLU)
  Output: 10-dim softmax   (critic weights summing to 1)

Total parameters: ~142,000
Inference time on CPU: < 0.5 ms  (negligible at 20 Hz)

Export to TorchScript at end of train.py:
  scripted = torch.jit.script(model)
  scripted.save('meta_critic/models/meta_critic.pt')
"""

import torch
import torch.nn as nn
import torch.nn.functional as F


class MetaCriticMLP(nn.Module):

    def __init__(
        self,
        input_dim:  int = 410,
        hidden:     list = None,
        output_dim: int = 10,
    ):
        super().__init__()
        if hidden is None:
            hidden = [256, 128, 64]

        layers = []
        prev = input_dim
        for h in hidden:
            layers.append(nn.Linear(prev, h))
            layers.append(nn.LayerNorm(h))
            layers.append(nn.ReLU())
            prev = h
        layers.append(nn.Linear(prev, output_dim))

        self.net = nn.Sequential(*layers)

        # Store dims for inspection
        self.input_dim  = input_dim
        self.output_dim = output_dim

        self._init_weights()

    def _init_weights(self):
        """Xavier uniform init for all linear layers."""
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.xavier_uniform_(m.weight)
                nn.init.zeros_(m.bias)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Args:
            x: (batch, 410) float32
        Returns:
            w: (batch, 10) float32  — softmax, sums to 1 per row
        """
        logits = self.net(x)
        return F.softmax(logits, dim=-1)

    @torch.jit.export
    def get_weights(self, x: torch.Tensor) -> torch.Tensor:
        """Alias used by TorchScript inference node."""
        return self.forward(x)

    def param_count(self) -> int:
        return sum(p.numel() for p in self.parameters())


def build_model(input_dim: int = 410, output_dim: int = 10) -> MetaCriticMLP:
    """Convenience factory used by train.py and evaluate.py."""
    return MetaCriticMLP(input_dim=input_dim, output_dim=output_dim)


if __name__ == '__main__':
    m = build_model()
    print(f'MetaCriticMLP  params: {m.param_count():,}')
    x = torch.randn(4, 410)
    w = m(x)
    print(f'Input:  {x.shape}')
    print(f'Output: {w.shape}  sum={w.sum(dim=-1)}')
    assert w.shape == (4, 10), 'shape mismatch'
    assert torch.allclose(w.sum(dim=-1), torch.ones(4), atol=1e-5), 'not summing to 1'
    print('All checks passed.')
