#!/usr/bin/env python3
"""
autobus sidewalk detection prototype.

Reads a recorded MP4 frame by frame, segments grass (left) / sidewalk / grass
(right), estimates the left and right sidewalk boundaries, and draws an
overlay (boundary lines + center path + steering error) so you can visually
sanity-check the approach before wiring it to the actual servo control loop.

Usage:
    # Play back with overlay, no file written
    python3 sidewalk_detect.py --input drive1.mp4

    # Write annotated output video for review
    python3 sidewalk_detect.py --input drive1.mp4 --output drive1_overlay.mp4

    # Open an interactive HSV tuner on a single frame first (recommended)
    python3 sidewalk_detect.py --input drive1.mp4 --tune

Controls during playback:
    q        quit
    space    pause / resume
    ,  .     step back / forward one frame while paused
"""

import argparse
import sys

import cv2
import numpy as np
import time


# ---------------------------------------------------------------------------
# Tunable defaults. These are a starting guess -- concrete sidewalk tends to
# be low-saturation, mid-to-high value (gray). Grass is green with higher
# saturation. Use --tune to dial these in against your actual footage before
# trusting the detector.
# ---------------------------------------------------------------------------
DEFAULT_SIDEWALK_HSV_LOW = (0, 0, 80)
DEFAULT_SIDEWALK_HSV_HIGH = (180, 60, 255)

DEFAULT_GRASS_HSV_LOW = (30, 40, 30)
DEFAULT_GRASS_HSV_HIGH = (95, 255, 255)

TRACKBAR_WINDOW = "HSV Tuner"


def build_sidewalk_mask(hsv, low, high):
    mask = cv2.inRange(hsv, np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8))
    # Clean up speckle noise and fill small holes so row-scanning is stable.
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def find_row_boundaries(mask, row_y):
    """
    For a single row of the binary sidewalk mask, find the leftmost and
    rightmost x where sidewalk pixels form a contiguous run through the
    image's horizontal center. Returns (left_x, right_x) or (None, None)
    if no sidewalk was found on that row.
    """
    row = mask[row_y]
    w = row.shape[0]
    center = w // 2

    # If center pixel isn't sidewalk, search outward a little for the
    # nearest sidewalk pixel to re-anchor on (handles the robot being
    # slightly off-center).
    if row[center] == 0:
        found = None
        for offset in range(1, w // 4):
            if center - offset >= 0 and row[center - offset] > 0:
                found = center - offset
                break
            if center + offset < w and row[center + offset] > 0:
                found = center + offset
                break
        if found is None:
            return None, None
        center = found

    left_x = center
    while left_x > 0 and row[left_x - 1] > 0:
        left_x -= 1

    right_x = center
    while right_x < w - 1 and row[right_x + 1] > 0:
        right_x += 1

    return left_x, right_x


def analyze_frame(frame, sidewalk_low, sidewalk_high, num_scan_rows=8, roi_top_frac=0.45):
    """
    Runs segmentation + row scanning on one frame.

    Returns:
        overlay: frame with drawn overlay (BGR, same size as input)
        result: dict with keys:
            'left_pts', 'right_pts': list of (x, y) boundary points found
            'center_offset_px': signed pixel offset of the mid-sidewalk path
                                 from the image's horizontal center at the
                                 bottom-most scan row (positive = path is to
                                 the right, i.e. steer right)
            'confidence': fraction of scan rows where sidewalk was detected
    """
    h, w = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = build_sidewalk_mask(hsv, sidewalk_low, sidewalk_high)

    roi_top = int(h * roi_top_frac)
    row_ys = np.linspace(h - 1, roi_top, num_scan_rows, dtype=int)

    left_pts, right_pts, center_pts = [], [], []
    for y in row_ys:
        l, r = find_row_boundaries(mask, y)
        if l is not None:
            left_pts.append((l, y))
            right_pts.append((r, y))
            center_pts.append(((l + r) // 2, y))

    overlay = frame.copy()

    # Shade the detected sidewalk mask in the ROI so you can see what the
    # color threshold is actually picking up.
    tint = np.zeros_like(frame)
    tint[:, :] = (0, 255, 255)
    mask_3ch = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    shaded = cv2.addWeighted(frame, 1.0, cv2.bitwise_and(tint, mask_3ch), 0.35, 0)
    overlay[roi_top:h] = shaded[roi_top:h]

    cv2.line(overlay, (0, roi_top), (w, roi_top), (255, 255, 0), 1)

    for pt in left_pts:
        cv2.circle(overlay, pt, 4, (0, 0, 255), -1)      # red = left edge
    for pt in right_pts:
        cv2.circle(overlay, pt, 4, (255, 0, 0), -1)      # blue = right edge
    for pt in center_pts:
        cv2.circle(overlay, pt, 3, (0, 255, 0), -1)      # green = path center

    # Fit straight lines through the left/right edge points (simple, robust
    # enough as a first pass -- swap for polyfit(deg=2) if the sidewalk
    # curves noticeably in your footage).
    def fit_and_draw(pts, color):
        if len(pts) < 2:
            return
        pts_arr = np.array(pts)
        vx, vy, x0, y0 = cv2.fitLine(pts_arr, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
        if abs(vy) < 1e-6:
            return
        y_top, y_bot = roi_top, h - 1
        t_top = (y_top - y0) / vy
        t_bot = (y_bot - y0) / vy
        x_top = int(x0 + vx * t_top)
        x_bot = int(x0 + vx * t_bot)
        cv2.line(overlay, (x_top, y_top), (x_bot, y_bot), color, 2)

    fit_and_draw(left_pts, (0, 0, 255))
    fit_and_draw(right_pts, (255, 0, 0))

    center_offset_px = None
    if center_pts:
        bottom_center_x = center_pts[0][0]  # row_ys is ordered bottom -> top
        center_offset_px = bottom_center_x - (w // 2)
        cv2.line(overlay, (w // 2, h), (w // 2, h - 30), (200, 200, 200), 1)
        cv2.putText(
            overlay,
            f"offset: {center_offset_px:+d}px",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0) if abs(center_offset_px) < 40 else (0, 0, 255),
            2,
        )

    confidence = len(left_pts) / float(num_scan_rows)
    cv2.putText(
        overlay,
        f"confidence: {confidence:.0%}",
        (10, 55),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        1,
    )

    return overlay, {
        "left_pts": left_pts,
        "right_pts": right_pts,
        "center_offset_px": center_offset_px,
        "confidence": confidence,
    }


def run_tuner(cap, sidewalk_low, sidewalk_high):
    """Interactive HSV threshold tuner on a single frame from the video."""
    ok, frame = cap.read()
    if not ok:
        print("Could not read a frame for tuning.")
        return sidewalk_low, sidewalk_high

    cv2.namedWindow(TRACKBAR_WINDOW)

    def nothing(_):
        pass

    cv2.createTrackbar("H low", TRACKBAR_WINDOW, sidewalk_low[0], 180, nothing)
    cv2.createTrackbar("H high", TRACKBAR_WINDOW, sidewalk_high[0], 180, nothing)
    cv2.createTrackbar("S low", TRACKBAR_WINDOW, sidewalk_low[1], 255, nothing)
    cv2.createTrackbar("S high", TRACKBAR_WINDOW, sidewalk_high[1], 255, nothing)
    cv2.createTrackbar("V low", TRACKBAR_WINDOW, sidewalk_low[2], 255, nothing)
    cv2.createTrackbar("V high", TRACKBAR_WINDOW, sidewalk_high[2], 255, nothing)

    print("Tuning window open. Adjust sliders so ONLY sidewalk is highlighted.")
    print("Press 'n' to advance to the next frame, 'q' to accept and continue.")

    while True:
        low = (
            cv2.getTrackbarPos("H low", TRACKBAR_WINDOW),
            cv2.getTrackbarPos("S low", TRACKBAR_WINDOW),
            cv2.getTrackbarPos("V low", TRACKBAR_WINDOW),
        )
        high = (
            cv2.getTrackbarPos("H high", TRACKBAR_WINDOW),
            cv2.getTrackbarPos("S high", TRACKBAR_WINDOW),
            cv2.getTrackbarPos("V high", TRACKBAR_WINDOW),
        )
        overlay, _ = analyze_frame(frame, low, high)
        cv2.imshow(TRACKBAR_WINDOW, overlay)

        key = cv2.waitKey(30) & 0xFF
        if key == ord("q"):
            cv2.destroyWindow(TRACKBAR_WINDOW)
            return low, high
        if key == ord("n"):
            ok, next_frame = cap.read()
            if ok:
                frame = next_frame


def main():
    parser = argparse.ArgumentParser(description="autobus sidewalk edge detection prototype")
    parser.add_argument("--input", required=True, help="path to recorded mp4")
    parser.add_argument("--output", default=None, help="optional path to write annotated mp4")
    parser.add_argument("--tune", action="store_true", help="open interactive HSV tuner first")
    parser.add_argument("--no-display", action="store_true", help="skip live preview window (headless)")
    parser.add_argument("--start-frame", type=int, default=0, help="skip ahead N frames before starting")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.input)
    if not cap.isOpened():
        print(f"Could not open {args.input}", file=sys.stderr)
        sys.exit(1)

    if args.start_frame:
        cap.set(cv2.CAP_PROP_POS_FRAMES, args.start_frame)

    sidewalk_low, sidewalk_high = DEFAULT_SIDEWALK_HSV_LOW, DEFAULT_SIDEWALK_HSV_HIGH

    if args.tune:
        sidewalk_low, sidewalk_high = run_tuner(cap, sidewalk_low, sidewalk_high)
        cap.set(cv2.CAP_PROP_POS_FRAMES, args.start_frame)
        print(f"Using HSV low={sidewalk_low} high={sidewalk_high}")

    writer = None
    if args.output:
        fps = cap.get(cv2.CAP_PROP_FPS) or 15
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(args.output, fourcc, fps, (w, h))

    paused = False
    frame = None

    while True:
        if not paused:
            ok, frame = cap.read()
            if not ok:
                break
            overlay, _ = analyze_frame(frame, sidewalk_low, sidewalk_high)
            if writer is not None:
                writer.write(overlay)
            if not args.no_display:
                cv2.imshow("autobus sidewalk detection", overlay)

        if not args.no_display:
            key = cv2.waitKey(1 if not paused else 0) & 0xFF
            if key == ord("q"):
                break
            elif key == ord(" "):
                paused = not paused
            elif key == ord(".") and paused:
                ok, frame = cap.read()
                if ok:
                    overlay, _ = analyze_frame(frame, sidewalk_low, sidewalk_high)
                    if writer is not None:
                        writer.write(overlay)
                    cv2.imshow("autobus sidewalk detection", overlay)
        time.sleep(.1)

    cap.release()
    if writer is not None:
        writer.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()