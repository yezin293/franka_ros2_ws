"""Capture checkerboard photos and compute camera intrinsics (camera_matrix,
distortion coefficients).

  /usr/bin/python3 scripts/calibrate_intrinsics.py --square-mm 25
  /usr/bin/python3 scripts/calibrate_intrinsics.py --cols 11 --rows 8 --square-mm 20

--cols/--rows are the board's SQUARE counts (not internal corners) -- e.g. an
"11x8" board has 11x8 squares and 10x7 internal corners, which is what
findChessboardCorners actually looks for.

A live preview window opens. Hold the printed checkerboard (see
scripts/checkerboard.png, or any other checkerboard you have) flat, fill
different parts of the frame and tilt it
at different angles across ~15-20 shots -- varied angles matter much more
than varied position for a good distortion estimate. Green corners = board
detected, press SPACE to capture that frame. Press ESC when done (need >=10
captures). Prints the reprojection error (RMS, pixels) at the end -- well
under 1.0 is good, over ~1.0 means redo with more/better-varied angles or a
flatter board.
"""
import argparse

import cv2
import numpy as np

OUT_PATH = 'scripts/camera_intrinsics.json'


def open_camera(device, width, height):
    cap = cv2.VideoCapture(device)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    for _ in range(5):  # first frame(s) after opening come back black
        cap.read()
    return cap


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', type=int, default=0)
    parser.add_argument('--width', type=int, default=3840)
    parser.add_argument('--height', type=int, default=2160)
    parser.add_argument('--cols', type=int, default=10, help='board squares, horizontal')
    parser.add_argument('--rows', type=int, default=7, help='board squares, vertical')
    parser.add_argument('--square-mm', type=float, required=True,
                         help='measured checkerboard square side length [mm]')
    parser.add_argument('--min-captures', type=int, default=10)
    args = parser.parse_args()
    pattern_size = (args.cols - 1, args.rows - 1)  # findChessboardCorners wants internal corners

    # Object points for one board, in (x, y, 0) board-plane coordinates, mm.
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= args.square_mm

    cap = open_camera(args.device, args.width, args.height)
    if not cap.isOpened():
        raise SystemExit(f'could not open camera /dev/video{args.device}')

    obj_points, img_points = [], []
    img_size = None
    print('SPACE to capture when corners are green, ESC when done '
          f'(need >= {args.min_captures})')
    cv2.namedWindow('calibrate_intrinsics', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('calibrate_intrinsics', 1280, 720)

    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        img_size = frame.shape[1::-1]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern_size)
        display = frame.copy()
        cv2.drawChessboardCorners(display, pattern_size, corners if found else None, found)
        cv2.putText(display, f'captured: {len(obj_points)}', (30, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0) if found else (0, 0, 255), 3)
        cv2.imshow('calibrate_intrinsics', display)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            break
        if key == 32 and found:  # SPACE
            refined = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            obj_points.append(objp)
            img_points.append(refined)
            print(f'captured {len(obj_points)}')

    cap.release()
    cv2.destroyAllWindows()

    if len(obj_points) < args.min_captures:
        raise SystemExit(f'only {len(obj_points)} captures, need >= {args.min_captures}')

    rms, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
        obj_points, img_points, img_size, None, None)
    print(f'\nRMS reprojection error: {rms:.3f} px (well under 1.0 is good)')
    print('camera_matrix:\n', camera_matrix)
    print('dist_coeffs:\n', dist_coeffs.ravel())

    import json
    with open(OUT_PATH, 'w') as f:
        json.dump({
            'image_width': img_size[0],
            'image_height': img_size[1],
            'camera_matrix': camera_matrix.tolist(),
            'dist_coeffs': dist_coeffs.ravel().tolist(),
            'rms_reprojection_error_px': rms,
        }, f, indent=2)
    print(f'saved to {OUT_PATH}')


if __name__ == '__main__':
    main()
