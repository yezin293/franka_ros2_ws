"""BC and ACT policies with the same ResNet18 backbone, inputs and outputs.

Inputs:  wrist RGB (ImageNet-normalised) + state (7 joints + gripper width, normalised)
Outputs: chunk x 8 = 7 normalised joint deltas (target - current joints) + 1 gripper logit (closed)

Both use the same loss (L1 on joints, BCE on the gripper), so the comparison
is about the architecture: single-step MLP vs. action-chunking CVAE transformer.
"""
import math

import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision

from common import ACTION_DIM, STATE_DIM


def resnet18_trunk(pretrained):
    weights = torchvision.models.ResNet18_Weights.IMAGENET1K_V1 if pretrained else None
    net = torchvision.models.resnet18(weights=weights)
    return nn.Sequential(*list(net.children())[:-2])   # B x 512 x H/32 x W/32


def action_loss(pred, target, is_pad):
    """pred/target: B x K x 8. Returns (loss, joint L1, gripper BCE)."""
    valid = (~is_pad).float().unsqueeze(-1)
    denom = valid.sum().clamp(min=1.0)
    joint_l1 = (F.l1_loss(pred[..., :7], target[..., :7], reduction='none') * valid).sum() / (denom * 7)
    grip_bce = (F.binary_cross_entropy_with_logits(
        pred[..., 7:], target[..., 7:], reduction='none') * valid).sum() / denom
    return joint_l1 + grip_bce, joint_l1, grip_bce


class BCPolicy(nn.Module):
    """ResNet18 -> global average pool, concat state -> MLP -> next action(s)."""

    def __init__(self, chunk=1, hidden=512, pretrained=True, dropout=0.1):
        super().__init__()
        self.chunk = chunk
        self.backbone = resnet18_trunk(pretrained)
        self.state_proj = nn.Sequential(nn.Linear(STATE_DIM, 64), nn.ReLU())
        self.head = nn.Sequential(
            nn.Linear(512 + 64, hidden), nn.ReLU(), nn.Dropout(dropout),
            nn.Linear(hidden, hidden), nn.ReLU(), nn.Dropout(dropout),
            nn.Linear(hidden, chunk * ACTION_DIM))

    def forward(self, image, state):
        feat = self.backbone(image).mean(dim=(2, 3))
        out = self.head(torch.cat([feat, self.state_proj(state)], dim=1))
        return out.view(-1, self.chunk, ACTION_DIM)

    def compute_loss(self, batch):
        pred = self(batch['image'], batch['state'])
        loss, joint_l1, grip_bce = action_loss(pred, batch['action'], batch['is_pad'])
        return loss, {'joint_l1': joint_l1, 'grip_bce': grip_bce}

    @torch.no_grad()
    def predict(self, image, state):
        return self(image, state)


def sine_2d(d_model, h, w, device):
    """Fixed 2D sin/cos position embedding, (h*w) x d_model."""
    half = d_model // 2
    freq = torch.exp(torch.arange(0, half, 2, device=device).float() * (-math.log(10000.0) / half))
    y = torch.arange(h, device=device).float()[:, None] * freq[None]
    x = torch.arange(w, device=device).float()[:, None] * freq[None]
    pe_y = torch.cat([y.sin(), y.cos()], dim=1)[:, None, :].expand(h, w, half)
    pe_x = torch.cat([x.sin(), x.cos()], dim=1)[None, :, :].expand(h, w, half)
    return torch.cat([pe_y, pe_x], dim=2).reshape(h * w, d_model)


class ACTPolicy(nn.Module):
    """Action Chunking with Transformers (Zhao et al. 2023), one camera.

    Training: a CVAE encoder reads (state, true action chunk) -> latent z.
    The policy reads image tokens + state + z and decodes the whole chunk.
    Inference: z = 0 (prior mean)."""

    def __init__(self, chunk=20, d_model=256, nhead=8, enc_layers=4, dec_layers=4,
                 ffn=1024, latent=32, dropout=0.1, kl_weight=10.0, pretrained=True):
        super().__init__()
        self.chunk = chunk
        self.latent = latent
        self.kl_weight = kl_weight
        self.backbone = resnet18_trunk(pretrained)
        self.input_proj = nn.Conv2d(512, d_model, kernel_size=1)

        def encoder(layers):
            layer = nn.TransformerEncoderLayer(d_model, nhead, ffn, dropout,
                                               batch_first=True, norm_first=True)
            return nn.TransformerEncoder(layer, layers, norm=nn.LayerNorm(d_model))

        # CVAE encoder over [CLS, state, a_1..a_K]
        self.cvae_encoder = encoder(enc_layers)
        self.cls = nn.Parameter(torch.zeros(1, 1, d_model))
        self.cvae_state = nn.Linear(STATE_DIM, d_model)
        self.cvae_action = nn.Linear(ACTION_DIM, d_model)
        self.cvae_pos = nn.Parameter(torch.randn(1, 2 + chunk, d_model) * 0.02)
        self.to_latent = nn.Linear(d_model, 2 * latent)

        # policy: encoder over [z, state, image tokens], decoder with K queries
        self.encoder = encoder(enc_layers)
        dec_layer = nn.TransformerDecoderLayer(d_model, nhead, ffn, dropout,
                                               batch_first=True, norm_first=True)
        self.decoder = nn.TransformerDecoder(dec_layer, dec_layers, norm=nn.LayerNorm(d_model))
        self.latent_proj = nn.Linear(latent, d_model)
        self.state_proj = nn.Linear(STATE_DIM, d_model)
        self.extra_pos = nn.Parameter(torch.randn(1, 2, d_model) * 0.02)
        self.queries = nn.Parameter(torch.randn(1, chunk, d_model) * 0.02)
        self.head = nn.Linear(d_model, ACTION_DIM)

    def encode_latent(self, state, action, is_pad):
        b = state.shape[0]
        tokens = torch.cat([self.cls.expand(b, -1, -1),
                            self.cvae_state(state)[:, None],
                            self.cvae_action(action)], dim=1) + self.cvae_pos
        mask = torch.cat([torch.zeros(b, 2, dtype=torch.bool, device=state.device), is_pad], dim=1)
        out = self.cvae_encoder(tokens, src_key_padding_mask=mask)[:, 0]
        mu, logvar = self.to_latent(out).chunk(2, dim=-1)
        return mu, logvar

    def forward(self, image, state, z):
        b = image.shape[0]
        feat = self.input_proj(self.backbone(image))         # B x d x h x w
        _, d, h, w = feat.shape
        img_tokens = feat.flatten(2).transpose(1, 2) + sine_2d(d, h, w, feat.device)[None]
        extra = torch.stack([self.latent_proj(z), self.state_proj(state)], dim=1) + self.extra_pos
        memory = self.encoder(torch.cat([extra, img_tokens], dim=1))
        out = self.decoder(self.queries.expand(b, -1, -1), memory)
        return self.head(out)                                # B x K x 8

    def compute_loss(self, batch):
        mu, logvar = self.encode_latent(
            batch['state'], batch['action'], batch['is_pad'])
        z = mu + torch.randn_like(mu) * (0.5 * logvar).exp()
        pred = self(batch['image'], batch['state'], z)
        loss, joint_l1, grip_bce = action_loss(pred, batch['action'], batch['is_pad'])
        kl = (-0.5 * (1 + logvar - mu.pow(2) - logvar.exp())).sum(-1).mean()
        return loss + self.kl_weight * kl, {'joint_l1': joint_l1, 'grip_bce': grip_bce, 'kl': kl}

    @torch.no_grad()
    def predict(self, image, state):
        z = torch.zeros(image.shape[0], self.latent, device=image.device)
        return self(image, state, z)


def build_policy(config):
    if config['policy'] == 'bc':
        return BCPolicy(chunk=config['chunk'], pretrained=config['pretrained'])
    return ACTPolicy(chunk=config['chunk'], kl_weight=config['kl_weight'],
                     pretrained=config['pretrained'])
