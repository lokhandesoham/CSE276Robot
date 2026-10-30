"""Build the HW2 printables: AprilTags (one per page) and a camera calibration checkerboard.

Usage (on the Mac, from the repo root):
    tools/.venv/bin/python tools/make_printables.py

Writes printables/apriltags.pdf and printables/checkerboard.pdf.
Print both at 100% / "Actual size" (never "Fit to page"), then check the 100 mm
scale bar on each page with a ruler before using them.

The tags are read from the repo's tag36_11_0000N.png files, decoded with OpenCV
to confirm the ID, and redrawn as vector squares so the print is sharp at any size.
"""
import pathlib
import subprocess
import tempfile

import cv2
import numpy as np


REPO = pathlib.Path(__file__).resolve().parent.parent
OUT_DIR = REPO / 'printables'
CHROME = '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome'

TAG_FILES = [REPO / f'tag36_11_0000{i}.png' for i in range(3)]
TAG_CELLS = 10           # tag36h11 image: 1 white border cell + 8 black-square cells + 1 white
BLACK_SQUARE_MM = 150.0  # edge of the black square; this is the "size" apriltag_ros needs
CELL_MM = BLACK_SQUARE_MM / 8

BOARD_COLS, BOARD_ROWS = 9, 7  # squares, so 8x6 inner corners
SQUARE_MM = 25.0


def read_tag(path):
    """Return the tag's ID and its 10x10 cell grid (1 = black)."""
    img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise SystemExit(f'cannot read {path}')
    h, w = img.shape
    cell = w // TAG_CELLS
    if h != w or cell * TAG_CELLS != w:
        raise SystemExit(f'{path.name}: expected a square image of {TAG_CELLS} cells, got {w}x{h}')

    # sample the centre of each cell
    centres = np.arange(TAG_CELLS) * cell + cell // 2
    cells = (img[np.ix_(centres, centres)] < 128).astype(int)

    # decode the original image to make sure it really is the tag we think it is
    padded = cv2.copyMakeBorder(img, cell, cell, cell, cell, cv2.BORDER_CONSTANT, value=255)
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
    _, ids, _ = detector.detectMarkers(padded)
    ids = [] if ids is None else np.asarray(ids).ravel()
    if len(ids) != 1:
        raise SystemExit(f'{path.name}: expected one tag36h11 tag, detected {len(ids)}')
    return int(ids[0]), cells


def scale_bar():
    return ('<div class="bar"><div class="line"></div>'
            '<span>this line must measure exactly 100 mm</span></div>')


def tag_page(tag_id, cells):
    rects = ''.join(
        f'<rect x="{c}" y="{r}" width="1.01" height="1.01"/>'
        for r in range(TAG_CELLS) for c in range(TAG_CELLS) if cells[r][c])
    side = TAG_CELLS * CELL_MM
    return f'''
<section class="page">
  <svg width="{side}mm" height="{side}mm" viewBox="0 0 {TAG_CELLS} {TAG_CELLS}" shape-rendering="crispEdges">{rects}</svg>
  <p class="label">tag36h11 &nbsp;ID {tag_id} &nbsp;&middot;&nbsp; black square = {BLACK_SQUARE_MM:.0f} mm edge to edge
     (measure it and use your measurement)</p>
  {scale_bar()}
</section>'''


def board_page():
    rects = ''.join(
        f'<rect x="{c}" y="{r}" width="1.01" height="1.01"/>'
        for r in range(BOARD_ROWS) for c in range(BOARD_COLS) if (r + c) % 2 == 0)
    return f'''
<section class="page">
  <svg width="{BOARD_COLS * SQUARE_MM}mm" height="{BOARD_ROWS * SQUARE_MM}mm"
       viewBox="0 0 {BOARD_COLS} {BOARD_ROWS}" shape-rendering="crispEdges">{rects}</svg>
  <p class="label">{BOARD_COLS}x{BOARD_ROWS} squares = {BOARD_COLS - 1}x{BOARD_ROWS - 1} inner corners
     &nbsp;&middot;&nbsp; square = {SQUARE_MM:.0f} mm. Mount flat on a rigid board.</p>
  {scale_bar()}
</section>'''


def html(pages, landscape):
    size = 'letter landscape' if landscape else 'letter'
    return f'''<!doctype html><html><head><meta charset="utf-8"><style>
@page {{ size: {size}; margin: 0; }}
html, body {{ margin: 0; background: #fff; }}
.page {{ height: {'215.9mm' if landscape else '279.4mm'}; display: flex; flex-direction: column;
         align-items: center; justify-content: center; page-break-after: always;
         font: 10pt Helvetica, Arial, sans-serif; color: #444; }}
.page:last-child {{ page-break-after: auto; }}
svg {{ fill: #000; display: block; }}
.label {{ margin: 6mm 0 3mm; }}
.bar {{ display: flex; flex-direction: column; align-items: center; }}
.bar .line {{ width: 100mm; height: 2mm; border: 0.3mm solid #444; border-top: none; box-sizing: border-box; }}
.bar span {{ font-size: 8pt; margin-top: 1mm; }}
</style></head><body>{''.join(pages)}</body></html>'''


def render(markup, pdf_path):
    with tempfile.NamedTemporaryFile('w', suffix='.html', delete=False) as f:
        f.write(markup)
    subprocess.run([CHROME, '--headless', '--disable-gpu', '--no-pdf-header-footer',
                    f'--print-to-pdf={pdf_path}', f.name],
                   check=True, capture_output=True)
    print(f'wrote {pdf_path.relative_to(REPO)}')


def main():
    OUT_DIR.mkdir(exist_ok=True)
    pages = []
    for path in TAG_FILES:
        tag_id, cells = read_tag(path)
        print(f'{path.name}: tag36h11 ID {tag_id}')
        pages.append(tag_page(tag_id, cells))
    render(html(pages, landscape=False), OUT_DIR / 'apriltags.pdf')
    render(html([board_page()], landscape=True), OUT_DIR / 'checkerboard.pdf')


if __name__ == '__main__':
    main()
