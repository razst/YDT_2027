import time
import cv2
import numpy as np

MIN_RADIUS = 10         
MIN_FILL = 0.70         
MIN_SOLIDITY = 0.60     
MAX_BALLS = 10
HOLD_FRAMES = 8         

SPEC_V_MIN = 230        
SPEC_S_MAX = 70         
SPEC_MAX_AREA = 2500    

state = {"ranges": [((5, 100, 60), (20, 255, 255))], "click": None}
LOG_FILE = "calibrated_hsv.txt"


def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        state["click"] = (x, y)


def make_ranges(h, s, v, hue_tol=12):
    """Build HSV ranges around a sampled color, wide on S and V for reflections."""
    h, s, v = int(h), int(s), int(v)
    s_lo = max(25, s - 100)
    v_lo = max(30, v - 120)
    lo, hi = h - hue_tol, h + hue_tol
    if lo < 0:  # hue wraps (reds)
        return [((0, s_lo, v_lo), (hi, 255, 255)), ((180 + lo, s_lo, v_lo), (179, 255, 255))]
    if hi > 179:
        return [((lo, s_lo, v_lo), (179, 255, 255)), ((0, s_lo, v_lo), (hi - 180, 255, 255))]
    return [((lo, s_lo, v_lo), (hi, 255, 255))]


def save_calibration_to_file(h, s, v, ranges):
    """HSV"""
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    try:
        with open(LOG_FILE, "a", encoding="utf-8") as f:
            f.write(f"[{timestamp}] HSV: H={int(h)}, S={int(s)}, V={int(v)} | Ranges: {ranges}\n")
        print(f"Saved calibration to {LOG_FILE}")
    except Exception as e:
        print(f"Error saving to file: {e}")


def remove_specular(frame):
    """Inpaint small bright/low-saturation highlights so they don't break the ball's mask."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    spec = ((hsv[..., 2] > SPEC_V_MIN) & (hsv[..., 1] < SPEC_S_MAX)).astype(np.uint8) * 255

    n, labels, stats, _ = cv2.connectedComponentsWithStats(spec, connectivity=8)
    small = np.zeros_like(spec)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] <= SPEC_MAX_AREA:
            small[labels == i] = 255

    small = cv2.dilate(small, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)), iterations=2)
    clean = cv2.inpaint(frame, small, 5, cv2.INPAINT_TELEA)
    return clean, small


def candidates_from_mask(mask):
    """Turn a binary mask into (x, y, r) ball candidates using hull-based shape checks."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found = []
    for c in contours:
        hull = cv2.convexHull(c)
        hull_area = cv2.contourArea(hull)
        if hull_area < 1:
            continue
        (x, y), r = cv2.minEnclosingCircle(hull)
        if r < MIN_RADIUS:
            continue
        fill = hull_area / (np.pi * r * r)
        solidity = cv2.contourArea(c) / hull_area
        if fill >= MIN_FILL and solidity >= MIN_SOLIDITY:
            found.append((hull_area, (int(x), int(y), int(r))))
    found.sort(reverse=True)
    return [f[1] for f in found[:MAX_BALLS]]


def detect_by_color(clean, ranges):
    blurred = cv2.GaussianBlur(clean, (9, 9), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in ranges:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15)))
    return candidates_from_mask(mask), mask


def detect_by_hough(clean):
    gray = cv2.cvtColor(clean, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 7)
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=60,
                             param1=100, param2=35,
                             minRadius=MIN_RADIUS, maxRadius=0)
    if circles is None:
        return []
    return [tuple(int(v) for v in np.round(c)) for c in circles[0][:MAX_BALLS]]


def update_tracks(tracks, detections):
    """Keep balls alive for a few frames when a reflection makes them flicker out."""
    new = [[x, y, r, 0] for (x, y, r) in detections]
    for x, y, r, age in tracks:
        matched = any(np.hypot(x - d[0], y - d[1]) < max(r, d[2]) * 1.5 for d in detections)
        if not matched and age < HOLD_FRAMES:
            new.append([x, y, r, age + 1])
    return new


def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Could not open camera")

    cv2.namedWindow("HSV Inspector")
    cv2.setMouseCallback("HSV Inspector", on_mouse)

    prev_time = time.perf_counter()

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame = cv2.flip(frame, 1)

        clean, _ = remove_specular(frame)

        if state["click"]:
            cx, cy = state["click"]
            state["click"] = None
            hsv_clean = cv2.cvtColor(clean, cv2.COLOR_BGR2HSV)
            patch = hsv_clean[max(0, cy - 5):cy + 6, max(0, cx - 5):cx + 6].reshape(-1, 3)
            h, s, v = np.median(patch, axis=0)
            state["ranges"] = make_ranges(h, s, v)
            print(f"Clicked HSV ~ ({int(h)}, {int(s)}, {int(v)})")
            save_calibration_to_file(h, s, v, state["ranges"])

        cv2.putText(frame, "Click anywhere to inspect HSV | Press 'q' to quit",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow("HSV Inspector", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()