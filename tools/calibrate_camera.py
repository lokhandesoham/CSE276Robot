"""Calibrate the camera from checkerboard photos and write robot_vision/config/camera_parameter.yaml.

Usage (on the Mac, from the repo root):
    tools/.venv/bin/python tools/calibrate_camera.py IMAGE_DIR
    tools/.venv/bin/python tools/calibrate_camera.py IMAGE_DIR --max-error 1.0   # drop bad shots, refit
    tools/.venv/bin/python tools/calibrate_camera.py IMAGE_DIR --dry-run         # don't write the YAML

IMAGE_DIR holds the frames saved by `ros2 run robot_control capture_frames` (raw, not rectified).
The board is printables/checkerboard.pdf: 8x6 inner corners, 25 mm squares.

Why projection_matrix = new camera matrix: the camera node undistorts with
getOptimalNewCameraMatrix(K, D, size, alpha=0) and publishes our projection_matrix as P,
and apriltag_ros computes tag poses from P alone (no distortion). So P has to be exactly
the matrix the rectified image was made with, or every tag distance is scaled wrong.
"""
import argparse
import pathlib

import cv2
import numpy as np


REPO = pathlib.Path(__file__).resolve().parent.parent
YAML_PATH = REPO / 'robot_vision' / 'config' / 'camera_parameter.yaml'

PATTERN = (8, 6)      # inner corners (cols, rows)
SQUARE_M = 0.025
EXTENSIONS = ('.ppm', '.png', '.jpg', '.jpeg')


def find_corners(paths):
    """Return (image_size, [(path, corners)]) for every image where the full board was found."""
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    size, found = None, []
    for path in paths:
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            print(f'  {path.name}: unreadable, skipped')
            continue
        if size is None:
            size = gray.shape[::-1]
        elif gray.shape[::-1] != size:
            raise SystemExit(f'{path.name} is {gray.shape[::-1]}, others are {size}: mixed resolutions')
        ok, corners = cv2.findChessboardCorners(
            gray, PATTERN, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            print(f'  {path.name}: board not found, skipped')
            continue
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        found.append((path, corners))
    return size, found


def calibrate(size, found, free_k3=False):
    board = np.zeros((PATTERN[0] * PATTERN[1], 3), np.float32)
    board[:, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2) * SQUARE_M
    object_points = [board] * len(found)
    image_points = [c for _, c in found]
    # k3 only matters near the image corners; unless the board was photographed there it
    # fits noise (first IMX219 run: same RMS, but k3 = -1.14 sent the corner pixel to x=-803)
    flags = 0 if free_k3 else cv2.CALIB_FIX_K3
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, size, None, None, flags=flags)

    per_image = []
    for (path, corners), rvec, tvec in zip(found, rvecs, tvecs):
        projected, _ = cv2.projectPoints(board, rvec, tvec, K, D)
        # flatten both: OpenCV versions disagree on (N, 1, 2) vs (N, 2)
        diff = projected.reshape(-1, 2) - corners.reshape(-1, 2)
        err = np.sqrt(np.mean(np.sum(diff ** 2, axis=1)))
        per_image.append((path, err))
    return rms, K, D, per_image


def coverage(size, found):
    """How much of the image the board corners reached, as a 3x3 grid of counts."""
    grid = np.zeros((3, 3), int)
    w, h = size
    for _, corners in found:
        for x, y in corners.reshape(-1, 2):
            grid[min(int(3 * y / h), 2), min(int(3 * x / w), 2)] += 1
    return grid


def write_yaml(path, size, K, D, P):
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    fs.write('image_width', size[0])
    fs.write('image_height', size[1])
    fs.write('camera_name', 'camera')
    fs.write('camera_matrix', K)
    fs.write('distortion_model', 'plumb_bob')
    fs.write('distortion_coefficients', D.reshape(1, -1))
    fs.write('rectification_matrix', np.eye(3))
    fs.write('projection_matrix', np.hstack([P, np.zeros((3, 1))]))
    fs.release()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('image_dir', type=pathlib.Path)
    parser.add_argument('--max-error', type=float,
                        help='drop images whose reprojection error (px) is above this, then refit')
    parser.add_argument('--dry-run', action='store_true', help="print results, don't write the YAML")
    parser.add_argument('--free-k3', action='store_true',
                        help='also fit k3 (only with many shots in the image corners; fixed at 0 by default)')
    args = parser.parse_args()

    paths = sorted(p for p in args.image_dir.iterdir() if p.suffix.lower() in EXTENSIONS)
    print(f'{len(paths)} images in {args.image_dir}')
    size, found = find_corners(paths)
    if len(found) < 10:
        raise SystemExit(f'board found in only {len(found)} images; need at least 10 (aim for 25+)')

    rms, K, D, per_image = calibrate(size, found, args.free_k3)
    if args.max_error is not None:
        keep = {p for p, e in per_image if e <= args.max_error}
        dropped = [p.name for p, _ in per_image if p not in keep]
        print(f'dropping {len(dropped)} images above {args.max_error} px: {", ".join(dropped) or "none"}')
        found = [(p, c) for p, c in found if p in keep]
        rms, K, D, per_image = calibrate(size, found, args.free_k3)

    print('\nper-image reprojection error (px), worst first:')
    for path, err in sorted(per_image, key=lambda pe: -pe[1]):
        print(f'  {path.name}: {err:.3f}')

    P, _ = cv2.getOptimalNewCameraMatrix(K, D, size, 0.0, size)
    hfov = np.degrees(2 * np.arctan(size[0] / (2 * P[0, 0])))
    vfov = np.degrees(2 * np.arctan(size[1] / (2 * P[1, 1])))

    print(f'\nused {len(found)} images at {size[0]}x{size[1]}')
    print(f'RMS reprojection error: {rms:.3f} px  ({"good" if rms < 0.5 else "OK" if rms < 1.0 else "poor, retake"})')
    print(f'K: fx={K[0, 0]:.1f} fy={K[1, 1]:.1f} cx={K[0, 2]:.1f} cy={K[1, 2]:.1f}')
    print(f'D: {np.array2string(D.ravel(), precision=4)}')
    print(f'P (rectified): fx={P[0, 0]:.1f} fy={P[1, 1]:.1f} cx={P[0, 2]:.1f} cy={P[1, 2]:.1f}'
          f'  -> FOV {hfov:.0f} x {vfov:.0f} deg')
    grid = coverage(size, found)
    print('board corner coverage (3x3 image grid; aim for no near-empty cells, especially corners):')
    for row in grid:
        print('  ' + ' '.join(f'{n:5d}' for n in row))

    # alpha=0 rectification is decided by the image corners, where the distortion model is
    # extrapolated if no board was photographed there; a lopsided P is the symptom
    aspect_change = abs((P[0, 0] / P[1, 1]) / (K[0, 0] / K[1, 1]) - 1)
    if aspect_change > 0.05 or grid[[0, 0, 2, 2], [0, 2, 0, 2]].min() < 2 * PATTERN[0]:
        print('WARNING: weak coverage of the image corners (rectified fx/fy differs from the raw by '
              f'{100 * aspect_change:.0f}%). Take more shots with the board in each image corner.')

    # an over-fitted model folds over near the edges: image corners undistort to wild places
    w, h = size
    corners = np.array([[[0, 0]], [[w - 1, 0]], [[0, h - 1]], [[w - 1, h - 1]]], np.float64)
    moved = np.abs(cv2.undistortPoints(corners, K, D, P=K).reshape(-1, 2) - corners.reshape(-1, 2)).max()
    if moved > 0.25 * w:
        print(f'WARNING: the distortion model moves an image corner by {moved:.0f} px, so it is not '
              'valid near the edges. Retake with the board in the corners (and drop --free-k3).')

    # save a before/after preview so straight lines can be checked by eye
    preview = cv2.imread(str(found[0][0]))
    rectified = cv2.undistort(preview, K, D, None, P)
    # next to the folder, not in it, so a rerun doesn't treat it as a calibration image
    preview_path = args.image_dir.resolve().parent / f'{args.image_dir.resolve().name}_rectified_preview.png'
    cv2.imwrite(str(preview_path), np.hstack([preview, rectified]))
    print(f'\nwrote {preview_path} (left: raw, right: rectified)')

    if args.dry_run:
        print('dry run, YAML not written')
    else:
        write_yaml(YAML_PATH, size, K, D, P)
        print(f'wrote {YAML_PATH.relative_to(REPO)}')


if __name__ == '__main__':
    main()
