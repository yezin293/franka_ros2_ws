"""Solve for the camera's pose in robot base frame via ArUco + solvePnP.

Takes one or more photos of the ArUco marker (robot clear of the frame,
marker moved to a different spot in the workspace for each extra photo --
see note below on why more than one position matters), detects its 4
corners in pixels per photo, pairs them with the matching 4 corners already
measured in robot-frame xyz (scripts/touch_points.json, recorded by
touching each corner with the closed gripper -- see record_touch_point.py),
and solves cv2.solvePnP(object_points=robot_xyz, image_points=pixel_xy,
camera_matrix, dist_coeffs) for the camera's pose directly in the robot
base frame.

  /usr/bin/python3 scripts/calibrate_extrinsics.py --session scripts/marker_photo.jpg ""
  /usr/bin/python3 scripts/calibrate_extrinsics.py \
      --session scripts/marker_photo.jpg "" \
      --session scripts/marker_photo_pos2.jpg "pos2_"

Why multiple positions: a single 25mm marker's 4 corners span only ~2.5cm,
which poorly constrains the camera's orientation -- solvePnP fits that tiny
cluster near-perfectly but small rotation error then blows up when
evaluated tens of cm away (e.g. at the trays). Touching the same marker at
2+ well-separated spots (each with its own photo) gives solvePnP a baseline
comparable to the actual workspace, which is what keeps extrapolated error
small. --session's second argument is a prefix (e.g. "pos2_") into
touch_points.json entries, so corner_cam_top_left + pos2_corner_cam_top_left
etc are read per session and merged into one solve.

Requires scripts/camera_intrinsics.json (calibrate_intrinsics_from_folder.py)
and scripts/touch_points.json with <prefix>corner_cam_top_left / _top_right /
_bottom_left / _bottom_right entries per session (record_touch_point.py).
Saves scripts/camera_extrinsics.json: rvec, tvec, and the reprojection error
of all corners (should be a few px at most -- large error means a touch
point or the marker ID/photo is off).
"""
import argparse
import json

import cv2
import numpy as np

MARKER_ID = 0
CORNER_SUFFIXES = ['corner_cam_top_left', 'corner_cam_top_right',
                    'corner_cam_bottom_left', 'corner_cam_bottom_right']


def classify_by_camera_view(corners_4x2):
    """Map cv2.aruco's 4 corners (TL,TR,BR,BL in the marker's own printed
    orientation) to the 4 points as they visually appear in THIS photo
    (top-left/top-right/bottom-left/bottom-right by pixel position), since
    the marker's printed orientation and its apparent rotation in-frame
    don't necessarily match."""
    cx, cy = corners_4x2.mean(axis=0)
    result = {}
    for x, y in corners_4x2:
        vert = 'top' if y < cy else 'bottom'
        horiz = 'left' if x < cx else 'right'
        result[f'corner_cam_{vert}_{horiz}'] = (x, y)
    return result


def detect_corners(photo_path):
    img = cv2.imread(photo_path)
    if img is None:
        raise SystemExit(f'could not read {photo_path}')
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None or MARKER_ID not in ids.flatten():
        raise SystemExit(f'marker id {MARKER_ID} not detected in {photo_path}')
    idx = list(ids.flatten()).index(MARKER_ID)
    raw_corners = corners[idx].reshape(4, 2).astype(np.float64)
    return classify_by_camera_view(raw_corners)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', nargs=2, action='append', required=True,
                         metavar=('PHOTO', 'PREFIX'),
                         help='one photo + touch_points.json name prefix; '
                              'repeat for each marker position')
    parser.add_argument('--intrinsics', default='scripts/camera_intrinsics.json')
    parser.add_argument('--touch-points', default='scripts/touch_points.json')
    parser.add_argument('--extra-point', nargs=3, action='append', default=[],
                         metavar=('PIXEL_X', 'PIXEL_Y', 'TOUCH_NAME'),
                         help='a manually-picked pixel (e.g. a bolt head) '
                              'paired with a touch_points.json entry, in '
                              'addition to the ArUco corners -- widens the '
                              'calibration baseline without printing more '
                              'markers. Must be from the SAME photo as the '
                              'first --session (same camera pose).')
    parser.add_argument('--out', default='scripts/camera_extrinsics.json')
    args = parser.parse_args()

    with open(args.touch_points) as f:
        touch_points = json.load(f)

    image_points = []
    object_points = []
    for px, py, name in args.extra_point:
        if name not in touch_points:
            raise SystemExit(f'touch_points.json missing: {name}')
        print(f'extra point {name}: ({float(px):.0f}, {float(py):.0f})')
        image_points.append((float(px), float(py)))
        object_points.append(touch_points[name]['xyz'])
    for photo_path, prefix in args.session:
        by_name = detect_corners(photo_path)
        print(f'{photo_path} (prefix {prefix!r}) detected pixel corners:')
        for suffix in CORNER_SUFFIXES:
            name = prefix + suffix
            if name not in touch_points:
                raise SystemExit(f'touch_points.json missing: {name}')
            px, py = by_name[suffix]
            print(f'  {name}: ({px:.0f}, {py:.0f})')
            image_points.append((px, py))
            object_points.append(touch_points[name]['xyz'])

    image_points = np.array(image_points, dtype=np.float64)
    object_points = np.array(object_points, dtype=np.float64)
    print(f'\n{len(object_points)} point correspondences total')

    with open(args.intrinsics) as f:
        intr = json.load(f)
    camera_matrix = np.array(intr['camera_matrix'])
    dist_coeffs = np.array(intr['dist_coeffs'])

    method = cv2.SOLVEPNP_ITERATIVE if len(object_points) >= 6 else cv2.SOLVEPNP_EPNP
    ok, rvec, tvec = cv2.solvePnP(object_points, image_points,
                                   camera_matrix, dist_coeffs, flags=method)
    if not ok:
        raise SystemExit('solvePnP failed')
    rvec, tvec = cv2.solvePnPRefineLM(object_points, image_points,
                                       camera_matrix, dist_coeffs, rvec, tvec)

    reprojected, _ = cv2.projectPoints(object_points, rvec, tvec,
                                        camera_matrix, dist_coeffs)
    reprojected = reprojected.reshape(-1, 2)
    errors = np.linalg.norm(reprojected - image_points, axis=1)
    print('\nreprojection error per point (px):', errors)
    print(f'mean reprojection error: {errors.mean():.2f} px '
          f'(max {errors.max():.2f} px)')

    camera_position_in_robot = (-cv2.Rodrigues(rvec)[0].T @ tvec).flatten()
    print('\ncamera position in robot base frame (x,y,z):', camera_position_in_robot)

    with open(args.out, 'w') as f:
        json.dump({
            'rvec': rvec.flatten().tolist(),
            'tvec': tvec.flatten().tolist(),
            'camera_position_in_robot_frame': camera_position_in_robot.tolist(),
            'mean_reprojection_error_px': float(errors.mean()),
            'max_reprojection_error_px': float(errors.max()),
        }, f, indent=2)
    print(f'\nsaved to {args.out}')


if __name__ == '__main__':
    main()
