"""Generate a printable checkerboard for camera intrinsic calibration.

  /usr/bin/python3 scripts/generate_checkerboard.py

Print scripts/checkerboard.png at 100% scale (NOT "fit to page"). After
printing, measure one square's actual side length with a ruler -- that's the
SQUARE_SIZE_MM calibrate_intrinsics.py needs (printers are rarely exactly
1:1). Mount it on something flat and rigid (a clipboard, a book cover) so it
doesn't warp during capture -- a warped board ruins the calibration.
"""
import cv2
import numpy as np

# "9x6" conventionally means 9x6 *internal* corners, i.e. a 10x7 square grid.
COLS, ROWS = 10, 7  # squares, not corners
NOMINAL_SQUARE_MM = 25
PIXELS_PER_MM = 10
OUT_PATH = 'scripts/checkerboard.png'


def main():
    sq = NOMINAL_SQUARE_MM * PIXELS_PER_MM
    board = np.zeros((ROWS * sq, COLS * sq), dtype=np.uint8)
    for r in range(ROWS):
        for c in range(COLS):
            if (r + c) % 2 == 0:
                board[r * sq:(r + 1) * sq, c * sq:(c + 1) * sq] = 255
    margin = sq // 2
    padded = cv2.copyMakeBorder(board, margin, margin, margin, margin,
                                 cv2.BORDER_CONSTANT, value=255)
    cv2.imwrite(OUT_PATH, padded)
    print(f'saved {OUT_PATH}')
    print(f'{COLS}x{ROWS} squares -> {COLS - 1}x{ROWS - 1} internal corners, '
          f'nominal square={NOMINAL_SQUARE_MM}mm')
    print('print at 100% scale, mount flat, then measure one square with a ruler.')


if __name__ == '__main__':
    main()
