"""Generate a printable ArUco marker of a known dictionary/ID.

  /usr/bin/python3 scripts/generate_aruco_marker.py --id 1 --out scripts/aruco_marker_1.png
  /usr/bin/python3 scripts/generate_aruco_marker.py --id 2 --out scripts/aruco_marker_2.png

Print at 100% scale (NOT "fit to page" -- that rescales it). The exact
printed size doesn't matter for calibrate_extrinsics.py (it uses robot-touch
measurements of the actual corners, not the nominal size), only that each
marker keeps a clear white margin and prints legibly.
"""
import argparse

import cv2

DICTIONARY = cv2.aruco.DICT_4X4_50
NOMINAL_SIZE_MM = 50
PIXELS_PER_MM = 10  # output image resolution; purely for print quality, not accuracy


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--id', type=int, default=0)
    parser.add_argument('--out', default='scripts/aruco_marker.png')
    args = parser.parse_args()

    aruco_dict = cv2.aruco.getPredefinedDictionary(DICTIONARY)
    side_px = NOMINAL_SIZE_MM * PIXELS_PER_MM
    img = cv2.aruco.generateImageMarker(aruco_dict, args.id, side_px)
    margin = side_px // 4
    padded = cv2.copyMakeBorder(img, margin, margin, margin, margin,
                                 cv2.BORDER_CONSTANT, value=255)
    cv2.imwrite(args.out, padded)
    print(f'saved {args.out}')
    print(f'dictionary=DICT_4X4_50, id={args.id}, nominal size={NOMINAL_SIZE_MM}mm')
    print('print at 100% scale (not "fit to page")')


if __name__ == '__main__':
    main()
