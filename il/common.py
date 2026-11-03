"""Dataset, normalisation and image augmentation shared by BC and ACT."""
import h5py
import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import Dataset

IMAGENET_MEAN = torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1)
IMAGENET_STD = torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1)
STATE_DIM = 8    # 7 joints + gripper width
ACTION_DIM = 8   # 7 joint targets + gripper command


def load_episodes(path):
    """All episodes of the HDF5 file, fully in RAM."""
    episodes = []
    with h5py.File(path, 'r') as f:
        meta = {'hz': float(f.attrs['hz']), 'image_hw': tuple(int(v) for v in f.attrs['image_hw'])}
        for name in sorted(f.keys()):
            g = f[name]
            episodes.append({
                'name': name,
                'images': g['images'][:],
                'state': np.concatenate(
                    [g['qpos'][:], g['gripper_width'][:][:, None]], axis=1).astype(np.float32),
                'action': g['action'][:].astype(np.float32),
            })
    return episodes, meta


def split_episodes(episodes, val_fraction, seed):
    rng = np.random.default_rng(seed)
    order = rng.permutation(len(episodes))
    n_val = max(1, int(round(len(episodes) * val_fraction)))
    val = sorted(order[:n_val].tolist())
    return ([episodes[i] for i in range(len(episodes)) if i not in val],
            [episodes[i] for i in val])


def compute_stats(episodes, chunk):
    """Normalisation for the state and for the joint actions.

    Joint actions are predicted relative to the current joints:
    delta_k = q_target(t+k) - q(t), k = 0..chunk-1. Absolute targets would
    need sub-0.3 deg precision just to beat "hold still" (the typical 10 Hz
    step), which a regression head does not reach. Gripper cmd stays 0..1.
    max_step: largest per-frame joint change in the demos, used as a speed
    limit when the policy runs on the robot."""
    state = np.concatenate([e['state'] for e in episodes])
    deltas = []
    for e in episodes:
        q, a = e['state'][:, :7], e['action'][:, :7]
        for k in range(min(chunk, len(a))):
            deltas.append(a[k:] - q[:len(q)-k])
    deltas = np.concatenate(deltas)
    steps = np.concatenate([np.abs(e['action'][:, :7] - e['state'][:, :7]) for e in episodes])
    return {
        'state_mean': state.mean(0), 'state_std': np.maximum(state.std(0), 1e-3),
        'delta_mean': deltas.mean(0), 'delta_std': np.maximum(deltas.std(0), 1e-4),
        'max_step': steps.max(0), 'p99_step': np.percentile(steps, 99, axis=0),
    }


def stats_to_torch(stats, device):
    return {k: torch.as_tensor(np.asarray(v), dtype=torch.float32, device=device)
            for k, v in stats.items()}


class ChunkDataset(Dataset):
    """(image, state, action chunk, pad mask) for every frame.

    chunk=1 gives classic single-step BC. Past the end of an episode the chunk
    is padded and masked out of the loss (as in ACT)."""

    def __init__(self, episodes, stats, chunk):
        self.episodes = episodes
        self.chunk = chunk
        self.index = [(e, t) for e, ep in enumerate(episodes) for t in range(len(ep['state']))]
        self.s_mean, self.s_std = stats['state_mean'], stats['state_std']
        self.d_mean, self.d_std = stats['delta_mean'], stats['delta_std']

    def __len__(self):
        return len(self.index)

    def __getitem__(self, i):
        e, t = self.index[i]
        ep = self.episodes[e]
        n = len(ep['action'])
        actions = ep['action'][t:t+self.chunk]
        pad = self.chunk - len(actions)
        is_pad = np.zeros(self.chunk, dtype=bool)
        if pad:
            actions = np.concatenate([actions, np.repeat(actions[-1:], pad, axis=0)])
            is_pad[self.chunk-pad:] = True
        actions = actions.copy()
        actions[:, :7] = (actions[:, :7] - ep['state'][t, :7] - self.d_mean) / self.d_std
        state = (ep['state'][t] - self.s_mean) / self.s_std
        return {
            'image': torch.from_numpy(ep['images'][t]).permute(2, 0, 1),  # uint8 3xHxW
            'state': torch.from_numpy(state.astype(np.float32)),
            'action': torch.from_numpy(actions.astype(np.float32)),
            'is_pad': torch.from_numpy(is_pad),
        }


def preprocess(images, augment):
    """uint8 Bx3xHxW (GPU) -> ImageNet-normalised float.

    augment: random crop of 90-100 % resized back to full size, plus mild
    brightness / contrast / saturation jitter. No flips: left/right decides
    which tray the cube is on."""
    x = images.float() / 255.0
    if augment:
        b, _, h, w = x.shape
        scale = torch.empty(b, device=x.device).uniform_(0.90, 1.0)
        tx = (1 - scale) * torch.empty(b, device=x.device).uniform_(-1, 1)
        ty = (1 - scale) * torch.empty(b, device=x.device).uniform_(-1, 1)
        theta = torch.zeros(b, 2, 3, device=x.device)
        theta[:, 0, 0] = scale
        theta[:, 1, 1] = scale
        theta[:, 0, 2] = tx
        theta[:, 1, 2] = ty
        grid = F.affine_grid(theta, x.shape, align_corners=False)
        x = F.grid_sample(x, grid, mode='bilinear', padding_mode='border', align_corners=False)
        bright = torch.empty(b, 1, 1, 1, device=x.device).uniform_(0.8, 1.2)
        contrast = torch.empty(b, 1, 1, 1, device=x.device).uniform_(0.8, 1.2)
        sat = torch.empty(b, 1, 1, 1, device=x.device).uniform_(0.8, 1.2)
        x = x * bright
        mean = x.mean(dim=(1, 2, 3), keepdim=True)
        x = (x - mean) * contrast + mean
        gray = x.mean(dim=1, keepdim=True)
        x = (x - gray) * sat + gray
        x = x.clamp(0, 1)
    return (x - IMAGENET_MEAN.to(x.device)) / IMAGENET_STD.to(x.device)
