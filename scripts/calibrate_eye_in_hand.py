"""Solve for the fixed flange->camera transform (eye-in-hand hand-eye
calibration), using the same "touch the checkerboard corners with the
gripper" trick as the old eye-to-hand calibration -- the board stays fixed,
only the wrist (and camera) moves across scripts/eih_samples/*.jpg.

  /usr/bin/python3 scripts/calibrate_eye_in_hand.py --cols 11 --rows 8

For each sample (photo + flange pose in scripts/eih_samples/poses.json):
  1. detect checkerboard corners, classify by camera-view position
  2. look up the matching eih_corner_cam_* touch points (robot-frame xyz)
     and bilinear-interpolate the rest (same as calibrate_extrinsics_checkerboard.py)
  3. solvePnP -> camera's pose in robot base frame at that sample
  4. combine with the flange's own pose (same sample) -> one estimate of the
     fixed flange->camera transform
All samples' estimates are then averaged (rotation via quaternion averaging,
translation via mean) and the spread across samples is reported as a
consistency check.
"""
import argparse
import glob
import json
import os

import cv2
import numpy as np

CORNER_NAMES = ['eih_top_left', 'eih_top_right', 'eih_bottom_left', 'eih_bottom_right']


def detect_grid(photo_path, cols, rows):
    img = cv2.imread(photo_path)
    if img is None:
        return None
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    pattern_size = (cols - 1, rows - 1)
    found, corners = cv2.findChessboardCorners(gray, pattern_size)
    if not found:
        return None
    corners = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
    pr, pc = pattern_size
    return corners.reshape(pc, pr, 2)


def classify_extremes(grid):
    pc, pr = grid.shape[0], grid.shape[1]
    extremes = {(0, 0): grid[0, 0], (0, pr - 1): grid[0, pr - 1],
                (pc - 1, 0): grid[pc - 1, 0], (pc - 1, pr - 1): grid[pc - 1, pr - 1]}
    pts = np.array(list(extremes.values()))
    cx, cy = pts.mean(axis=0)
    result = {}
    for (j, i), (x, y) in extremes.items():
        vert = 'top' if y < cy else 'bottom'
        horiz = 'left' if x < cx else 'right'
        result[f'eih_{vert}_{horiz}'] = (j, i)
    return result


def interpolated_object_points(grid, extremes, corner_xyz):
    pc, pr = grid.shape[0], grid.shape[1]
    j_top, i_left = extremes['eih_top_left']
    j_bottom, i_right = extremes['eih_bottom_right']
    i_denom = (i_right - i_left) or 1
    j_denom = (j_bottom - j_top) or 1
    image_points, object_points = [], []
    for j in range(pc):
        for i in range(pr):
            u = (i - i_left) / i_denom
            v = (j - j_top) / j_denom
            p = ((1 - u) * (1 - v) * corner_xyz['eih_top_left']
                 + u * (1 - v) * corner_xyz['eih_top_right']
                 + (1 - u) * v * corner_xyz['eih_bottom_left']
                 + u * v * corner_xyz['eih_bottom_right'])
            image_points.append(grid[j, i])
            object_points.append(p)
    return np.array(image_points, dtype=np.float64), np.array(object_points, dtype=np.float64)


def _quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def average_quaternions(quats):
    quats = np.array(quats)
    ref = quats[0]
    for i in range(len(quats)):
        if np.dot(quats[i], ref) < 0:
            quats[i] = -quats[i]
    M = quats.T @ quats
    eigvals, eigvecs = np.linalg.eigh(M)
    return eigvecs[:, np.argmax(eigvals)]


def R_to_quat(R):
    w = np.sqrt(max(0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.sqrt(max(0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    y = np.sqrt(max(0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    z = np.sqrt(max(0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    x = np.copysign(x, R[2, 1] - R[1, 2])
    y = np.copysign(y, R[0, 2] - R[2, 0])
    z = np.copysign(z, R[1, 0] - R[0, 1])
    return np.array([x, y, z, w])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cols', type=int, required=True)
    parser.add_argument('--rows', type=int, required=True)
    parser.add_argument('--samples-dir', default='scripts/eih_samples')
    parser.add_argument('--touch-points', default='scripts/touch_points.json')
    parser.add_argument('--intrinsics', default='scripts/eih_camera_intrinsics.json')
    parser.add_argument('--out', default='scripts/eih_extrinsics.json')
    args = parser.parse_args()

    with open(args.touch_points) as f:
        touch_points = json.load(f)
    missing = [n for n in CORNER_NAMES if n not in touch_points]
    if missing:
        raise SystemExit(f'touch_points.json missing: {missing}')
    corner_xyz = {n: np.array(touch_points[n]['xyz']) for n in CORNER_NAMES}

    with open(args.intrinsics) as f:
        intr = json.load(f)
    camera_matrix = np.array(intr['camera_matrix'])
    dist_coeffs = np.array(intr['dist_coeffs'])

    with open(os.path.join(args.samples_dir, 'poses.json')) as f:
        flange_poses = json.load(f)

    translations = []
    quats = []
    per_sample_report = []
    for idx, pose in sorted(flange_poses.items()):
        photo_path = os.path.join(args.samples_dir, f'{idx}.jpg')
        grid = detect_grid(photo_path, args.cols, args.rows)
        if grid is None:
            print(f'[{idx}] checkerboard not found, skipping')
            continue
        extremes = classify_extremes(grid)
        image_points, object_points = interpolated_object_points(grid, extremes, corner_xyz)

        ok, rvec, tvec = cv2.solvePnP(object_points, image_points,
                                       camera_matrix, dist_coeffs,
                                       flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            print(f'[{idx}] solvePnP failed, skipping')
            continue
        reproj, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
        err = np.linalg.norm(reproj.reshape(-1, 2) - image_points, axis=1).mean()

        R_board_to_cam, _ = cv2.Rodrigues(rvec)
        t_board_to_cam = tvec.flatten()
        # camera pose in robot base (board/robot-base frame == same frame,
        # since object_points were already given in robot-base frame)
        R_base_to_cam = R_board_to_cam
        t_base_to_cam = t_board_to_cam
        R_cam_to_base = R_base_to_cam.T
        t_cam_to_base = -R_cam_to_base @ t_base_to_cam

        flange_xyz = np.array(pose['xyz'])
        flange_quat = pose['quat_xyzw']
        R_flange_to_base = _quat_to_R(*flange_quat)
        R_base_to_flange = R_flange_to_base.T
        t_base_to_flange = -R_base_to_flange @ flange_xyz

        # flange -> camera = (base -> flange) . (camera -> base)
        R_flange_to_cam = R_base_to_flange @ R_cam_to_base
        t_flange_to_cam = R_base_to_flange @ t_cam_to_base + t_base_to_flange

        translations.append(t_flange_to_cam)
        quats.append(R_to_quat(R_flange_to_cam))
        per_sample_report.append((idx, err, t_flange_to_cam))
        print(f'[{idx}] reprojection error {err:.2f}px  '
              f'flange->camera t={t_flange_to_cam.round(4)}')

    if len(translations) < 3:
        raise SystemExit(f'only {len(translations)} usable samples, need at least 3-5')

    translations = np.array(translations)
    quats = np.array(quats)
    # Robust outlier rejection: planar solvePnP occasionally finds a flipped
    # pose that still reprojects with low error but is physically wrong
    # (translation way outside plausible camera-mount range). Reject samples
    # far from the per-axis median before averaging.
    median = np.median(translations, axis=0)
    mad = np.median(np.abs(translations - median), axis=0) + 1e-6
    keep = (np.abs(translations - median) < 8 * mad).all(axis=1)
    if (~keep).any():
        dropped = [per_sample_report[i][0] for i in range(len(keep)) if not keep[i]]
        print(f'\ndropping outlier samples (likely pose-flip ambiguity): {dropped}')
    translations = translations[keep]
    quats = quats[keep]

    t_mean = translations.mean(axis=0)
    t_std = translations.std(axis=0)
    q_mean = average_quaternions(quats)

    print(f'\n{len(translations)}/{len(flange_poses)} samples used')
    print(f'flange->camera translation mean: {t_mean.round(4)}  std: {t_std.round(4)}')
    print(f'flange->camera quaternion (xyzw): {q_mean.round(4)}')

    with open(args.out, 'w') as f:
        json.dump({
            'flange_to_camera_xyz': t_mean.tolist(),
            'flange_to_camera_xyz_std': t_std.tolist(),
            'flange_to_camera_quat_xyzw': q_mean.tolist(),
            'num_samples_used': len(translations),
            'num_samples_total': len(flange_poses),
        }, f, indent=2)
    print(f'saved to {args.out}')


if __name__ == '__main__':
    main()
