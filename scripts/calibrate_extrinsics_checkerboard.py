"""Solve for the camera's pose in robot base frame using a checkerboard
instead of (or in addition to) the ArUco marker -- many more, more widely
spread correspondence points than a single small marker, without printing
anything new.

Only the checkerboard's 4 extreme internal corners need to be touched with
the robot (same amount of work as the marker's 4 corners). The other
internal corners' robot-frame positions are filled in by bilinear
interpolation across those 4 touched corners (valid since the board is
rigid and flat), then ALL corners (touched + interpolated) go into one
solvePnP call -- far better conditioned than 4 points spanning 2.5cm.

Step 1 -- detect corners and get a labeled photo to touch from:
  /usr/bin/python3 scripts/calibrate_extrinsics_checkerboard.py label \
      --photo scripts/checkerboard_photo.jpg --cols 11 --rows 8

This prints each extreme corner's pixel position by camera-view color (same
blue/yellow/green/red convention as the marker) and saves a labeled crop to
check. Touch each with record_touch_point.py as cb_top_left / cb_top_right /
cb_bottom_left / cb_bottom_right (or pass --prefix for a different name).

Step 2 -- solve once all 4 are touched:
  /usr/bin/python3 scripts/calibrate_extrinsics_checkerboard.py solve \
      --photo scripts/checkerboard_photo.jpg --cols 11 --rows 8
"""
import argparse
import json

import cv2
import numpy as np


def detect_grid(photo_path, cols, rows):
    img = cv2.imread(photo_path)
    if img is None:
        raise SystemExit(f'could not read {photo_path}')
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    pattern_size = (cols - 1, rows - 1)  # (points per row, points per col)
    found, corners = cv2.findChessboardCorners(gray, pattern_size)
    if not found:
        raise SystemExit(f'checkerboard not found in {photo_path} '
                          f'(cols={cols}, rows={rows})')
    corners = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
    pr, pc = pattern_size
    grid = corners.reshape(pc, pr, 2)  # grid[j, i] = (x, y) pixel
    return img, grid


def classify_extremes(grid):
    """Return {camera-view name: (j, i)} for the grid's 4 extreme corners,
    same top/bottom/left/right-by-pixel-position convention as the marker
    code, so orientation in the photo doesn't matter."""
    pc, pr = grid.shape[0], grid.shape[1]
    extremes = {(0, 0): grid[0, 0], (0, pr - 1): grid[0, pr - 1],
                (pc - 1, 0): grid[pc - 1, 0], (pc - 1, pr - 1): grid[pc - 1, pr - 1]}
    pts = np.array(list(extremes.values()))
    cx, cy = pts.mean(axis=0)
    result = {}
    for (j, i), (x, y) in extremes.items():
        vert = 'top' if y < cy else 'bottom'
        horiz = 'left' if x < cx else 'right'
        result[f'{vert}_{horiz}'] = (j, i)
    return result


def cmd_label(args):
    img, grid = detect_grid(args.photo, args.cols, args.rows)
    extremes = classify_extremes(grid)
    colors = {'top_left': ((255, 0, 0), 'BLUE'), 'top_right': ((0, 255, 255), 'YELLOW'),
              'bottom_left': ((0, 255, 0), 'GREEN'), 'bottom_right': ((0, 0, 255), 'RED')}
    print('touch these with record_touch_point.py as:')
    for name, (j, i) in extremes.items():
        x, y = grid[j, i]
        color, cname = colors[name]
        print(f'  {cname:6s} ({x:.0f},{y:.0f}) -> --name {args.prefix}{name}')
        cv2.circle(img, (int(x), int(y)), 18, color, -1)
        cv2.circle(img, (int(x), int(y)), 22, (0, 0, 0), 3)
    small = cv2.resize(img, (img.shape[1] // 2, img.shape[0] // 2))
    cv2.imwrite(args.label_out, small)
    print(f'labeled photo saved to {args.label_out}')


def cmd_solve(args):
    img, grid = detect_grid(args.photo, args.cols, args.rows)
    extremes = classify_extremes(grid)

    with open(args.touch_points) as f:
        touch_points = json.load(f)
    missing = [args.prefix + n for n in extremes if args.prefix + n not in touch_points]
    if missing:
        raise SystemExit(f'touch_points.json missing: {missing}')
    corner_xyz = {name: np.array(touch_points[args.prefix + name]['xyz'])
                  for name in extremes}

    pc, pr = grid.shape[0], grid.shape[1]
    j_top, i_left = extremes['top_left']
    j_bottom, i_right = extremes['bottom_right']
    # u: 0 at i_left -> 1 at i_right (handles either index order)
    # v: 0 at j_top -> 1 at j_bottom
    i_denom = (i_right - i_left) or 1
    j_denom = (j_bottom - j_top) or 1

    image_points = []
    object_points = []
    for j in range(pc):
        for i in range(pr):
            u = (i - i_left) / i_denom
            v = (j - j_top) / j_denom
            p = ((1 - u) * (1 - v) * corner_xyz['top_left']
                 + u * (1 - v) * corner_xyz['top_right']
                 + (1 - u) * v * corner_xyz['bottom_left']
                 + u * v * corner_xyz['bottom_right'])
            image_points.append(grid[j, i])
            object_points.append(p)
    image_points = np.array(image_points, dtype=np.float64)
    object_points = np.array(object_points, dtype=np.float64)
    print(f'{len(object_points)} corner correspondences (4 touched, '
          f'{len(object_points) - 4} interpolated)')

    with open(args.intrinsics) as f:
        intr = json.load(f)
    camera_matrix = np.array(intr['camera_matrix'])
    dist_coeffs = np.array(intr['dist_coeffs'])

    ok, rvec, tvec = cv2.solvePnP(object_points, image_points,
                                   camera_matrix, dist_coeffs,
                                   flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise SystemExit('solvePnP failed')
    rvec, tvec = cv2.solvePnPRefineLM(object_points, image_points,
                                       camera_matrix, dist_coeffs, rvec, tvec)

    reprojected, _ = cv2.projectPoints(object_points, rvec, tvec,
                                        camera_matrix, dist_coeffs)
    errors = np.linalg.norm(reprojected.reshape(-1, 2) - image_points, axis=1)
    print(f'reprojection error: mean={errors.mean():.2f}px '
          f'max={errors.max():.2f}px (over all {len(errors)} corners)')

    camera_position_in_robot = (-cv2.Rodrigues(rvec)[0].T @ tvec).flatten()
    print('camera position in robot base frame (x,y,z):', camera_position_in_robot)

    with open(args.out, 'w') as f:
        json.dump({
            'rvec': rvec.flatten().tolist(),
            'tvec': tvec.flatten().tolist(),
            'camera_position_in_robot_frame': camera_position_in_robot.tolist(),
            'mean_reprojection_error_px': float(errors.mean()),
            'max_reprojection_error_px': float(errors.max()),
            'num_points': len(errors),
        }, f, indent=2)
    print(f'saved to {args.out}')


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest='cmd', required=True)
    for name, fn in [('label', cmd_label), ('solve', cmd_solve)]:
        p = sub.add_parser(name)
        p.add_argument('--photo', required=True)
        p.add_argument('--cols', type=int, required=True)
        p.add_argument('--rows', type=int, required=True)
        p.add_argument('--prefix', default='cb_')
        p.add_argument('--touch-points', default='scripts/touch_points.json')
        p.add_argument('--intrinsics', default='scripts/camera_intrinsics.json')
        p.add_argument('--out', default='scripts/camera_extrinsics.json')
        p.add_argument('--label-out', default='scripts/checkerboard_labeled.jpg')
        p.set_defaults(func=fn)
    args = parser.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
