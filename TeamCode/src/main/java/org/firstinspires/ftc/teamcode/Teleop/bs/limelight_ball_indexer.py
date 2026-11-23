# =====================================================================================
#  Limelight 3A (goBILDA Smart Camera) Python SnapScript: BIOBUZZ slide-chamber indexer
# =====================================================================================
#
#  WHAT IT DOES
#  The camera looks into the slide chamber. Each frame, this script finds every pollen
#  (yellow) and nectar (red or blue) ball, sorts them from the DROP GATE to the far end,
#  and sends the order to the robot in the "llpython" array. The TeleOp reads it with
#  LLResult.getPythonOutput() (CAM_MODE = 0).
#
#  OUTPUT (8 numbers, always the same length so the Java side can trust the format)
#    llpython = [1, count, s0, s1, s2, s3, 0, 0]
#      1      heartbeat: proves this script ran (the Java ignores frames without it)
#      count  number of balls seen, 0..4
#      s0..s3 ball codes, s0 = ball CLOSEST TO THE DROP GATE
#             0 empty, 1 pollen, 2 red nectar, 3 blue nectar
#
#  SETUP (Limelight web UI, http://limelight.local:5801 with the camera on USB)
#    1. Pick a pipeline slot (default 0 = CAM_PIPELINE in the TeleOp), set type to "Python".
#    2. Paste this file in and save.
#    3. Edit the TUNE block below while watching the live image:
#       - ROI: draw the crop around the chamber only, so field elements never count.
#       - AXIS / GATE_AT_MIN: which way the chamber runs in the image and where the gate is.
#         The overlay numbers each ball 0,1,2,3: 0 must be the ball against the drop gate.
#       - Ball widths in pixels: measure one pollen and one nectar along the chamber.
#       - HSV ranges: adjust if balls are missed or the background is picked up.
#
#  WHY A COLOUR SCRIPT (and not a neural network)?
#  The chamber view never changes: same angle, distance and lighting. Colour thresholds are
#  very reliable there and run fast on the Limelight's processor. A neural detector is the
#  alternative (CAM_MODE = 1 in the TeleOp) if lighting makes colour unreliable.
# =====================================================================================

import cv2
import numpy as np

# ---------------- TUNE ----------------
AXIS = 'x'             # chamber runs along image 'x' (left-right) or 'y' (up-down)
GATE_AT_MIN = True     # True: drop gate at the LEFT (x) or TOP (y) of the image. False: right / bottom.
ROI = (0, 0, 640, 480) # crop to the chamber: x0, y0, x1, y1 in full-resolution pixels
SCALE = 0.5            # analyse at half resolution: ~4x less work, higher frame rate, plenty
                       #   of detail for balls this big. Set 1.0 if small balls get missed.
MIN_AREA = 150         # blobs smaller than this (full-res px^2) are noise, not balls
POLLEN_DIAM_PX = 60    # pollen width along the chamber, full-res px (ball is ~2.8")
NECTAR_DIAM_PX = 77    # nectar width along the chamber, full-res px (ball is ~3.6", ~1.29x pollen)
MAX_BALLS = 4          # chamber capacity; must match MAX in the TeleOp

# HSV colour ranges. OpenCV hue runs 0-180 (not 0-360). Format: ((H, S, V) low, (H, S, V) high).
YEL = ((18, 90, 90), (35, 255, 255))        # pollen yellow
RED1 = ((0, 120, 70), (8, 255, 255))        # red sits at BOTH ends of the hue circle,
RED2 = ((170, 120, 70), (180, 255, 255))    #   so it needs two ranges joined together
BLU = ((100, 120, 60), (130, 255, 255))     # blue nectar
# --------------------------------------

KERNEL = np.ones((3, 3), np.uint8)   # small cleanup filter for speckle (3x3 suits half resolution)
DRAW = {1: (0, 255, 255), 2: (0, 0, 255), 3: (255, 0, 0)}   # overlay dot colours (BGR) per code
EMPTY = np.array([[]])               # Limelight requires a contour return value; we don't use it


def find(mask, code, diam, out, off):
    """Find balls of one colour and append (position, code) for each to `out`.

    mask: black/white image, white = pixels of this colour
    code: 1 pollen, 2 red nectar, 3 blue nectar
    diam: this ball's width in FULL-res pixels (used to split merged blobs)
    off:  ROI offset along the axis, so positions are in full-image pixels
    """
    # "Opening" erases specks smaller than the kernel without shrinking real balls.
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
    # [-2] picks the contour list on both OpenCV 3 (3 return values) and 4 (2 return values).
    for c in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]:
        if cv2.contourArea(c) < MIN_AREA * SCALE * SCALE:   # area threshold scaled to the shrunk image
            continue
        x, y, w, h = cv2.boundingRect(c)
        # Measure the blob along the chamber direction only.
        start, length = (x, w) if AXIS == 'x' else (y, h)
        # KEY STEP: two touching balls of the same colour look like ONE long blob.
        # Dividing the blob length by one ball's width gives how many balls it really is,
        # e.g. a 120 px yellow blob with 60 px pollen = 2 pollen.
        n = min(MAX_BALLS, max(1, int(round(length / (diam * SCALE)))))
        step = length / float(n)
        for k in range(n):
            # Centre of the k-th ball inside the blob, converted back to full-image pixels.
            out.append((off + (start + step * (k + 0.5)) / SCALE, code))


def runPipeline(image, llrobot):
    """Called by the Limelight for every camera frame. Must return (contour, image, llpython)."""
    x0, y0, x1, y1 = ROI
    # Crop first (less to process), then shrink. INTER_AREA is the cleanest shrink method.
    small = cv2.resize(image[y0:y1, x0:x1], None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_AREA)
    # HSV separates colour (hue) from brightness, so thresholds survive lighting changes.
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    off = x0 if AXIS == 'x' else y0

    balls = []   # (position along chamber in full-res px, code)
    find(cv2.inRange(hsv, YEL[0], YEL[1]), 1, POLLEN_DIAM_PX, balls, off)
    find(cv2.inRange(hsv, RED1[0], RED1[1]) | cv2.inRange(hsv, RED2[0], RED2[1]), 2, NECTAR_DIAM_PX, balls, off)
    find(cv2.inRange(hsv, BLU[0], BLU[1]), 3, NECTAR_DIAM_PX, balls, off)

    # Sort gate-first. Different colours are found separately, so this sort is what
    # interleaves them into the true order in the chamber.
    balls.sort(key=lambda b: b[0], reverse=not GATE_AT_MIN)
    balls = balls[:MAX_BALLS]   # more than 4 means noise got through; keep the 4 nearest the gate

    llpython = [1, len(balls), 0, 0, 0, 0, 0, 0]

    # Debug overlay in the Limelight web UI: ROI box, then a coloured dot + index per ball.
    # Use it to check that ball 0 is the one against the drop gate.
    cy, cx = (y0 + y1) // 2, (x0 + x1) // 2
    cv2.rectangle(image, (x0, y0), (x1 - 1, y1 - 1), (255, 255, 255), 1)
    for i, (pos, code) in enumerate(balls):
        llpython[2 + i] = code
        p = (int(pos), cy) if AXIS == 'x' else (cx, int(pos))
        cv2.circle(image, p, 8, DRAW[code], -1)
        cv2.putText(image, str(i), (p[0] + 10, p[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    return EMPTY, image, llpython
