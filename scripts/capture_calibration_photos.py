"""Capture checkerboard photos one at a time, no live preview (faster/simpler
than calibrate_intrinsics.py's GUI loop).

  /usr/bin/python3 scripts/capture_calibration_photos.py --n 20

Move the board to a new position/angle, press Enter, repeat. Saves to
scripts/calib_photos/00.jpg, 01.jpg, ... Then run
calibrate_intrinsics_from_folder.py on that folder.
"""
import argparse
import os

import cv2


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
    parser.add_argument('--n', type=int, default=20)
    parser.add_argument('--outdir', default='scripts/calib_photos')
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    cap = open_camera(args.device, args.width, args.height)
    if not cap.isOpened():
        raise SystemExit(f'could not open camera /dev/video{args.device}')

    print(f'move the checkerboard to a new position/angle, then press Enter '
          f'({args.n} photos). Ctrl+C to stop early.')
    count = 0
    try:
        while count < args.n:
            input(f'[{count + 1}/{args.n}] Enter to capture... ')
            for _ in range(3):  # flush a couple of stale frames
                cap.read()
            ok, frame = cap.read()
            if not ok:
                print('  read failed, retrying')
                continue
            path = os.path.join(args.outdir, f'{count:02d}.jpg')
            cv2.imwrite(path, frame)
            print(f'  saved {path}')
            count += 1
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
    print(f'\n{count} photos saved to {args.outdir}')


if __name__ == '__main__':
    main()
