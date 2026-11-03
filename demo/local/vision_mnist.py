"""
Webcam frame -> (colour, digit) for the demo.

FrameClassifier asks Qwen2.5-VL first (vision_qwen.py). 
"""
from collections import Counter, deque
import os

import numpy as np
import cv2
import onnxruntime as ort

# Qwen is optional; the fallback works without transformers/torch
try:
    from vision_qwen import QwenClassifier
    _HAS_QWEN_MODULE = True
except ImportError:
    QwenClassifier = None
    _HAS_QWEN_MODULE = False


# OpenCV hue runs 0..179, so red sits at both ends
COLOUR_PROTOTYPES = {
    "RED":     {"hue": 0,   "hue_alt": 179},
    "GREEN":   {"hue": 60,  "hue_alt": None},
    "BLUE":    {"hue": 110, "hue_alt": None},
    "MAGENTA": {"hue": 150, "hue_alt": None},
}

HUE_TOL = 30

# ignore glare (high V) and grey (low S) pixels when reading the paper colour
GLARE_V = 250
GREY_S = 30
# low saturation and high value means white paper
WHITE_S_MAX = 50
WHITE_V_MIN = 150


def default_roi(frame_shape, frac=0.55):
    h, w = frame_shape[:2]
    side = int(min(w, h) * frac)
    return (w - side) // 2, (h - side) // 2, side, side


def draw_guide(frame, roi, label="HOLD CARD HERE"):
    x, y, w, h = roi
    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 255), 2)
    cv2.putText(frame, label, (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


def classify_colour(frame_bgr, roi):
    """Median HSV of the ROI's outer 15% ring, matched to the nearest hue."""
    x, y, w, h = roi
    patch = frame_bgr[y:y+h, x:x+w]
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)

    ring = np.zeros((h, w), dtype=np.uint8)
    border = int(0.15 * min(h, w))
    ring[:border, :] = 255; ring[-border:, :] = 255
    ring[:, :border] = 255; ring[:, -border:] = 255

    S = hsv[:, :, 1]
    V = hsv[:, :, 2]

    # white has no hue and would be dropped by the grey filter, so check it first
    ring_mask = ring > 0
    if ring_mask.sum() > 50:
        S_ring_med = float(np.median(S[ring_mask]))
        V_ring_med = float(np.median(V[ring_mask]))
        if S_ring_med < WHITE_S_MAX and V_ring_med > WHITE_V_MIN:
            return "WHITE", (0.0, S_ring_med, V_ring_med)

    valid = (V < GLARE_V) & (S > GREY_S)
    mask = ring_mask & valid

    if mask.sum() < 50:
        # nothing usable in the ring; report the plain mean and no colour
        mean = cv2.mean(hsv, mask=ring)
        return None, (mean[0], mean[1], mean[2])

    pixels = hsv[mask]
    H_med = float(np.median(pixels[:, 0]))
    S_med = float(np.median(pixels[:, 1]))
    V_med = float(np.median(pixels[:, 2]))

    def hue_dist(h1, h2):
        d = abs(h1 - h2)
        return min(d, 180 - d)

    best_name, best_dist = None, 999.0
    for name, p in COLOUR_PROTOTYPES.items():
        candidates = [p["hue"]]
        if p["hue_alt"] is not None:
            candidates.append(p["hue_alt"])
        d = min(hue_dist(H_med, c) for c in candidates)
        if d < best_dist:
            best_name, best_dist = name, d

    if best_dist > HUE_TOL:
        return None, (H_med, S_med, V_med)
    return best_name, (H_med, S_med, V_med)


# the board only knows 1-7; 0, 8 and 9 are rejected
TRAINED_DIGITS = {"D1", "D2", "D3", "D4", "D5", "D6", "D7"}

MNIST_MEAN = 0.1307
MNIST_STD  = 0.3081

# accept a prediction if it is confident enough or clearly ahead of the runner-up
MIN_CONFIDENCE = 0.40
MIN_MARGIN = 0.15


class MnistClassifier:
    """LeNet ONNX model run on a 28x28 crop of the ROI."""

    def __init__(self, onnx_path, min_confidence=MIN_CONFIDENCE,
                 min_margin=MIN_MARGIN):
        if not os.path.isfile(onnx_path):
            raise FileNotFoundError(
                f"{onnx_path} not found. Run train_mnist.py first to "
                f"produce this file.")
        self.session = ort.InferenceSession(
            onnx_path, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.min_confidence = min_confidence
        self.min_margin = min_margin

    def _preprocess(self, patch_bgr):
        """Threshold, crop the digit, return a normalised 1x1x28x28 array or None."""
        h, w = patch_bgr.shape[:2]

        gray = cv2.cvtColor(patch_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        # block 21 / C 8 suit digits about 80-200 px tall. Use a bigger block
        # for thick strokes, a smaller C if strokes come out broken.
        bw = cv2.adaptiveThreshold(
            gray, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            21, 8)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(
            bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        areas = [cv2.contourArea(c) for c in contours]
        max_a = max(areas)
        if max_a < 0.005 * h * w:
            return None

        # digits 1-7 are one stroke each, so take the largest contour that
        # isn't tiny, ROI-sized or touching the edge. The rest is usually card
        # edges, phone bezels or fingers.
        EDGE_MARGIN = 3
        valid = []
        for c, a in zip(contours, areas):
            if a < 0.05 * max_a:
                continue
            cx, cy, cw, ch = cv2.boundingRect(c)
            if cw > 0.85 * w or ch > 0.85 * h:
                continue
            if (cx < EDGE_MARGIN or cy < EDGE_MARGIN or
                cx + cw > w - EDGE_MARGIN or cy + ch > h - EDGE_MARGIN):
                continue
            valid.append((a, cx, cy, cw, ch))
        if not valid:
            return None
        valid.sort(key=lambda t: -t[0])
        _, xs, ys, bw_, bh_ = valid[0]
        xe, ye = xs + bw_, ys + bh_

        # pad to a square with an MNIST-like margin, then shrink to 28x28
        crop = bw[ys:ye, xs:xe]
        ch, cw = crop.shape
        side = max(ch, cw)
        pad_total = int(side * 0.28)
        square = np.zeros((side + pad_total, side + pad_total), dtype=np.uint8)
        y_off = (square.shape[0] - ch) // 2
        x_off = (square.shape[1] - cw) // 2
        square[y_off:y_off + ch, x_off:x_off + cw] = crop

        img28 = cv2.resize(square, (28, 28), interpolation=cv2.INTER_AREA)
        x = img28.astype(np.float32) / 255.0
        x = (x - MNIST_MEAN) / MNIST_STD
        return x[None, None, :, :]

    def infer(self, patch_bgr):
        """Returns (digit name or None, confidence, top class)."""
        x = self._preprocess(patch_bgr)
        if x is None:
            return None, 0.0, -1

        logits = self.session.run(None, {self.input_name: x})[0][0]
        e = np.exp(logits - logits.max())
        probs = e / e.sum()

        order = np.argsort(-probs)
        top = int(order[0])
        conf = float(probs[top])
        second = float(probs[order[1]])
        margin = conf - second

        if conf < self.min_confidence and margin < self.min_margin:
            return None, conf, top

        if top == 0:
            return None, conf, top
        name = f"D{top}"
        if name not in TRAINED_DIGITS:
            return None, conf, top
        return name, conf, top


# majority vote over the last few frames, for continuous mode
class Smoother:
    def __init__(self, window=8, min_votes=5):
        self.window = window
        self.min_votes = min_votes
        self.buf = deque(maxlen=window)

    def update(self, label):
        self.buf.append(label)
        if len(self.buf) < self.window:
            return None
        top_label, top_count = Counter(self.buf).most_common(1)[0]
        if top_label is not None and top_count >= self.min_votes:
            return top_label
        return None


class FrameClassifier:
    """Qwen first, then the HSV + LeNet fallback.

    onnx_path is required. qwen_model_path=None (or a missing folder)
    disables Qwen. qwen_device is 'auto', 'cuda' or 'cpu'.
    """

    def __init__(self, onnx_path, qwen_model_path=None,
                 qwen_device="auto", min_confidence=MIN_CONFIDENCE):
        self.digit_clf = MnistClassifier(onnx_path, min_confidence)

        self.qwen = None
        if qwen_model_path and _HAS_QWEN_MODULE:
            if os.path.isdir(qwen_model_path):
                self.qwen = QwenClassifier(qwen_model_path, device=qwen_device)
                # weights load on first use
            else:
                print(f"[vision] vlm path {qwen_model_path} not found, "
                      f"running fallback-only")
        elif qwen_model_path and not _HAS_QWEN_MODULE:
            print(f"[vision] primary backend not importable (missing deps?), "
                  f"running fallback-only")

    def _try_qwen(self, patch_bgr):
        """(colour, digit, conf, extra) from Qwen, or None."""
        if self.qwen is None:
            print("[dbg] vlm: not loaded")
            return None
        try:
            result = self.qwen.classify(patch_bgr)
        except Exception as e:
            import traceback
            print(f"[vlm] inference error: {e}")
            traceback.print_exc()
            return None

        if result is not None:
            print(f"[dbg vlm] raw='{result.get('raw_text', '')}' "
                  f"colour={result.get('colour')} digit={result.get('digit')} "
                  f"latency_ms={result.get('latency_ms', 0):.0f}")
        else:
            print("[dbg vlm] classify() returned None")

        if result is None:
            return None
        colour = result.get("colour")
        digit = result.get("digit")
        conf = result.get("confidence", 0.0)
        if colour is None or digit is None:
            return None
        extra = {
            "raw_text": result.get("raw_text", ""),
            "latency_ms": result.get("latency_ms", 0.0),
        }
        return colour, digit, conf, extra

    def _try_fallback(self, frame_bgr, roi):
        colour, hsv = classify_colour(frame_bgr, roi)
        x, y, w, h = roi
        patch = frame_bgr[y:y+h, x:x+w]
        digit, conf, top = self.digit_clf.infer(patch)
        return colour, digit, conf, hsv, top

    def classify_frame(self, frame_bgr, roi=None):
        """Dict with colour, digit, confidence, top_class, hsv, roi and
        backend ('vlm' or 'fallback')."""
        if roi is None:
            roi = default_roi(frame_bgr.shape)
        x, y, w, h = roi
        patch = frame_bgr[y:y+h, x:x+w]

        qwen_result = self._try_qwen(patch)
        if qwen_result is not None:
            colour, digit, conf, extra = qwen_result
            # HSV for the overlay, whichever backend answered
            _, hsv = classify_colour(frame_bgr, roi)
            return {
                "colour": colour, "digit": digit,
                "confidence": conf, "top_class": -1,
                "hsv": hsv, "roi": roi,
                "backend": "vlm",
                "qwen_raw": extra["raw_text"],
                "qwen_latency_ms": extra["latency_ms"],
            }

        colour, digit, conf, hsv, top = self._try_fallback(frame_bgr, roi)
        return {
            "colour": colour, "digit": digit,
            "confidence": conf, "top_class": top,
            "hsv": hsv, "roi": roi,
            "backend": "fallback",
        }
