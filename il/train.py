#!/usr/bin/env python3
"""Train a BC or ACT policy on data/il/*.hdf5.

  python3 il/train.py --policy bc
  python3 il/train.py --policy act
  python3 il/train.py --policy bc --no-pretrained      # backbone from scratch

Both share the same data split (by episode, --seed), augmentation, backbone
and loss, so their validation numbers are directly comparable. Checkpoints
and a CSV log go to runs/<policy>_<time>/ (best.pt = lowest val joint error).
"""
import argparse
import csv
import json
import time
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader

from common import ChunkDataset, compute_stats, load_episodes, preprocess, split_episodes
from models import build_policy

DEFAULTS = {
    'bc': {'chunk': 1, 'batch': 64, 'lr': 1e-4},
    'act': {'chunk': 20, 'batch': 32, 'lr': 1e-4},
}


def to_device(batch, device):
    return {k: v.to(device, non_blocking=True) for k, v in batch.items()}


@torch.no_grad()
def evaluate(policy, loader, stats, device):
    """Validation with the deployment-time forward pass (ACT: z = 0).
    joint_err_rad: mean |error| of the first predicted step, in radians.
    grip_acc: first-step gripper command accuracy."""
    policy.eval()
    err_sum = torch.zeros(7, device=device)
    correct = count = 0
    a_std = torch.as_tensor(stats['delta_std'], device=device)
    for batch in loader:
        batch = to_device(batch, device)
        with torch.autocast('cuda', dtype=torch.bfloat16):
            pred = policy.predict(preprocess(batch['image'], False), batch['state']).float()
        first, target = pred[:, 0], batch['action'][:, 0]
        err_sum += ((first[:, :7] - target[:, :7]).abs() * a_std).sum(0)
        correct += ((first[:, 7] > 0) == (target[:, 7] > 0.5)).sum().item()
        count += len(first)
    policy.train()
    per_joint = (err_sum / count).cpu().numpy()
    return {'joint_err_rad': float(per_joint.mean()), 'grip_acc': correct / count,
            'per_joint_err_rad': per_joint.tolist()}


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--policy', choices=['bc', 'act'], required=True)
    parser.add_argument('--data', type=Path, default=Path('data/il/demos_10hz_192x256.hdf5'))
    parser.add_argument('--out', type=Path, default=None)
    parser.add_argument('--steps', type=int, default=30000)
    parser.add_argument('--batch', type=int, default=None)
    parser.add_argument('--lr', type=float, default=None)
    parser.add_argument('--backbone-lr', type=float, default=1e-5)
    parser.add_argument('--chunk', type=int, default=None, help='bc default 1, act default 20')
    parser.add_argument('--kl-weight', type=float, default=10.0)
    parser.add_argument('--no-pretrained', action='store_true')
    parser.add_argument('--no-augment', action='store_true')
    parser.add_argument('--val-fraction', type=float, default=0.1)
    parser.add_argument('--val-every', type=int, default=1000)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--workers', type=int, default=6)
    args = parser.parse_args()
    for key, value in DEFAULTS[args.policy].items():
        if getattr(args, key) is None:
            setattr(args, key, value)
    if args.out is None:
        args.out = Path('runs') / f'{args.policy}_{time.strftime("%Y%m%d_%H%M%S")}'
    args.out.mkdir(parents=True, exist_ok=True)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    device = 'cuda'

    episodes, data_meta = load_episodes(args.data)
    train_eps, val_eps = split_episodes(episodes, args.val_fraction, args.seed)
    stats = compute_stats(train_eps, args.chunk)
    print(f'{len(train_eps)} train / {len(val_eps)} val episodes, '
          f'{sum(len(e["state"]) for e in train_eps)} train frames')
    hold = np.concatenate([np.abs(e['action'][:, :7] - e['state'][:, :7]) for e in val_eps]).mean()
    print(f'reference: "hold current joints" val joint err = {np.degrees(hold):.3f} deg')

    config = {
        'policy': args.policy, 'chunk': args.chunk, 'kl_weight': args.kl_weight,
        'pretrained': not args.no_pretrained, 'augment': not args.no_augment,
        'hz': data_meta['hz'], 'image_hw': data_meta['image_hw'],
        'data': str(args.data), 'val_episodes': [e['name'] for e in val_eps],
        'args': {k: str(v) for k, v in vars(args).items()},
    }
    (args.out / 'config.json').write_text(json.dumps(config, indent=2))

    train_loader = DataLoader(ChunkDataset(train_eps, stats, args.chunk), batch_size=args.batch,
                              shuffle=True, num_workers=args.workers, drop_last=True,
                              pin_memory=True, persistent_workers=True)
    val_loader = DataLoader(ChunkDataset(val_eps, stats, args.chunk), batch_size=128,
                            num_workers=2)

    policy = build_policy(config).to(device)
    backbone = [p for n, p in policy.named_parameters() if n.startswith('backbone.')]
    others = [p for n, p in policy.named_parameters() if not n.startswith('backbone.')]
    optimizer = torch.optim.AdamW([{'params': others, 'lr': args.lr},
                                   {'params': backbone, 'lr': args.backbone_lr}],
                                  weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.LambdaLR(
        optimizer, lambda s: min(1.0, (s+1)/500) * 0.5 * (1 + np.cos(np.pi * s / args.steps)))
    print(f'{sum(p.numel() for p in policy.parameters())/1e6:.1f} M parameters')

    log = open(args.out / 'log.csv', 'w', newline='')
    writer = csv.writer(log)
    writer.writerow(['step', 'loss', 'joint_l1', 'grip_bce', 'kl',
                     'val_joint_err_rad', 'val_grip_acc', 'seconds'])
    best = float('inf')
    step, start = 0, time.time()
    running = {}
    while step < args.steps:
        for batch in train_loader:
            batch = to_device(batch, device)
            batch['image'] = preprocess(batch['image'], not args.no_augment)
            with torch.autocast('cuda', dtype=torch.bfloat16):
                loss, parts = policy.compute_loss(batch)
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(policy.parameters(), 10.0)
            optimizer.step()
            scheduler.step()
            step += 1
            for k, v in {'loss': loss, **parts}.items():
                running[k] = running.get(k, 0.0) + float(v)
            if step % args.val_every == 0 or step == args.steps:
                means = {k: v / args.val_every for k, v in running.items()}
                running = {}
                val = evaluate(policy, val_loader, stats, device)
                elapsed = time.time() - start
                writer.writerow([step, means['loss'], means['joint_l1'], means['grip_bce'],
                                 means.get('kl', ''), val['joint_err_rad'], val['grip_acc'],
                                 round(elapsed)])
                log.flush()
                print(f'step {step:6d}  loss {means["loss"]:.4f}  '
                      f'val joint err {np.degrees(val["joint_err_rad"]):.3f} deg  '
                      f'grip acc {val["grip_acc"]:.3f}  ({elapsed/60:.1f} min)')
                checkpoint = {'config': config, 'stats': stats, 'model': policy.state_dict(),
                              'step': step, 'val': val}
                torch.save(checkpoint, args.out / 'last.pt')
                if val['joint_err_rad'] < best:
                    best = val['joint_err_rad']
                    torch.save(checkpoint, args.out / 'best.pt')
            if step >= args.steps:
                break
    log.close()
    print(f'done. best val joint err {np.degrees(best):.3f} deg -> {args.out}/best.pt')


if __name__ == '__main__':
    main()
