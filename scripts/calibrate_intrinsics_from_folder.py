"""Compute camera intrinsics from a folder of checkerboard photos (no GUI).

  /usr/bin/python3 scripts/calibrate_intrinsics_from_folder.py --cols 11 --rows 8 --square-mm 16

Reads every .jpg/.png in --folder (default scripts/calib_photos), finds the
checkerboard in each, and runs cv2.calibrateCamera. Prints which photos
failed (bad angle/blur/board not fully visible -- fine to have a few, just
need >= --min-captures good ones).
"""
import argparse
import glob
import json
import os

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--folder', default='scripts/calib_photos')
    parser.add_argument('--cols', type=int, default=10, help='board squares, horizontal')
    parser.add_argument('--rows', type=int, default=7, help='board squares, vertical')
    parser.add_argument('--square-mm', type=float, required=True)
    parser.add_argument('--min-captures', type=int, default=10)
    parser.add_argument('--out', default='scripts/camera_intrinsics.json')
    args = parser.parse_args()
    pattern_size = (args.cols - 1, args.rows - 1)

    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= args.square_mm

    paths = sorted(glob.glob(os.path.join(args.folder, '*.jpg'))
                    + glob.glob(os.path.join(args.folder, '*.png')))
    if not paths:
        raise SystemExit(f'no photos found in {args.folder}')

    obj_points, img_points = [], []
    img_size = None
    for path in paths:
        img = cv2.imread(path)
        if img is None:
            print(f'{path}: could not read, skipping')
            continue
        img_size = img.shape[1::-1]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern_size)
        if not found:
            print(f'{path}: board NOT found')
            continue
        refined = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        obj_points.append(objp)
        img_points.append(refined)
        print(f'{path}: OK')

    print(f'\n{len(obj_points)}/{len(paths)} photos usable')
    if len(obj_points) < args.min_captures:
        raise SystemExit(f'only {len(obj_points)} usable, need >= {args.min_captures} '
                          '-- take more photos (cover frame edges/corners, vary angle)')

    rms, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
        obj_points, img_points, img_size, None, None)
    print(f'\nRMS reprojection error: {rms:.3f} px (well under 1.0 is good)')
    print('camera_matrix:\n', camera_matrix)
    print('dist_coeffs:\n', dist_coeffs.ravel())

    with open(args.out, 'w') as f:
        json.dump({
            'image_width': img_size[0],
            'image_height': img_size[1],
            'camera_matrix': camera_matrix.tolist(),
            'dist_coeffs': dist_coeffs.ravel().tolist(),
            'rms_reprojection_error_px': rms,
        }, f, indent=2)
    print(f'saved to {args.out}')


if __name__ == '__main__':
    main()
