"""
Live HippocamFLEX demo: classify a card from the webcam, send it to the
board, and draw the recall on the video.

    python3 demo_mnist.py --mock                      # no board
    python3 demo_mnist.py --tcp-host 192.168.2.100    # real board
    python3 demo_mnist.py --mock --qwen-path ''       # LeNet fallback only
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# so the sibling modules import from any working directory
sys.path.insert(0, str(Path(__file__).resolve().parent))
import hippocamflex_hv as hv
from transport import make_transport, InferResult

import vision_mnist as vision


FG = (255, 255, 255)
DIM = (180, 180, 180)
OK_GREEN = (80, 220, 120)
ERR_RED = (80, 80, 240)
HIGHLIGHT = (0, 220, 255)
BACKEND_COLOUR = {"vlm": (0, 230, 230), "fallback": (160, 160, 160)}
FONT = cv2.FONT_HERSHEY_SIMPLEX


def overlay_status(frame, colour, digit, hsv_triplet, confidence,
                   show_live, backend=None):
    if show_live:
        lines = [
            f"Colour : {colour if colour else '---'}",
            (f"Digit  : {digit if digit else '---'}  (conf {confidence*100:4.1f}%)"
             if confidence else "Digit  : ---"),
        ]
    else:
        lines = ["Colour : (waiting for capture)",
                 "Digit  : (waiting for capture)"]
    if hsv_triplet is not None:
        H, S, V = hsv_triplet
        lines.append(f"HSV    : ({H:5.1f}, {S:5.1f}, {V:5.1f})")
    if backend:
        lines.append(f"Backend: {backend}")
    y = 30
    for s in lines:
        cv2.putText(frame, s, (15, y), FONT, 0.7, FG, 2)
        y += 28


def overlay_capture_result(frame, colour, digit, confidence, backend=None):
    h, w = frame.shape[:2]
    x0 = (w // 2) - 180; y0 = 20
    box_w, box_h = 360, 105 if backend else 90
    cv2.rectangle(frame, (x0, y0), (x0 + box_w, y0 + box_h), (40, 40, 40), -1)
    cv2.rectangle(frame, (x0, y0), (x0 + box_w, y0 + box_h), HIGHLIGHT, 2)
    cv2.putText(frame, "LAST CAPTURE", (x0 + 10, y0 + 22),
                FONT, 0.55, HIGHLIGHT, 2)
    cv2.putText(frame, f"Colour : {colour or '---'}",
                (x0 + 10, y0 + 50), FONT, 0.6, FG, 2)
    digit_line = f"Digit  : {digit or '---'}"
    if confidence:
        digit_line += f"  ({confidence*100:.1f}%)"
    cv2.putText(frame, digit_line, (x0 + 10, y0 + 75), FONT, 0.6, FG, 2)
    if backend:
        c = BACKEND_COLOUR.get(backend, FG)
        cv2.putText(frame, f"via {backend}", (x0 + 10, y0 + 98),
                    FONT, 0.5, c, 1)


def overlay_prediction(frame, result: InferResult):
    h, w = frame.shape[:2]
    x0, y0 = w - 340, 140
    box_w, box_h = 325, 180
    cv2.rectangle(frame, (x0, y0), (x0 + box_w, y0 + box_h), (40, 40, 40), -1)
    cv2.rectangle(frame, (x0, y0), (x0 + box_w, y0 + box_h), FG, 1)

    title_colour = OK_GREEN if result.ok else ERR_RED
    cv2.putText(frame, "FPGA:HippocamFLEX prediction", (x0 + 10, y0 + 25),
                FONT, 0.6, title_colour, 2)

    lines = []
    if result.ok:
        lines.append(f"Present (VP)  : {result.present}")
        lines.append(f"Next    (VF1) : {result.future1 or '---'}")
        lines.append(f"Then    (VF2) : {result.future2 or '---'}")
        # CA1 timestamp 0 is the furthest future, so sort descending to read
        # present -> future
        tagged = [name for name, v in result.all_events.items()
                  if v in (0x40, 0x80, 0xC0)]      # present, VF1, VF2
        if tagged:
            ordered = sorted(tagged, key=lambda n: result.timestamps.get(n, 999),
                             reverse=True)
            seq = " -> ".join(ordered)
            lines.append(f"Theta Sequ : {seq}")
    else:
        lines.append("No prediction")
        lines.append(result.note[:40])

    y = y0 + 55
    for s in lines:
        cv2.putText(frame, s, (x0 + 10, y), FONT, 0.58, FG, 1)
        y += 24


def overlay_prompt(frame, mode):
    h, w = frame.shape[:2]
    cv2.rectangle(frame, (0, h - 95), (w, h - 40), (25, 25, 25), -1)

    if mode == "triggered":
        line1 = "Show a card (digit 1-7, on RED/GREEN/BLUE/MAGENTA/WHITE)"
        line2 = "SPACE/ENTER: capture + show ROI debug  -  Q/ESC: quit"
    else:
        line1 = "Continuous mode: classification runs on every frame"
        line2 = "Q/ESC to quit  -  SPACE to force re-query last stable pair"
    cv2.putText(frame, line1, (15, h - 68), FONT, 0.65, FG, 2)
    cv2.putText(frame, line2, (15, h - 48), FONT, 0.55, DIM, 1)


def overlay_banner(frame, text):
    h, w = frame.shape[:2]
    cv2.rectangle(frame, (0, h - 40), (w, h), (30, 30, 30), -1)
    cv2.putText(frame, text, (15, h - 12), FONT, 0.6, FG, 2)


def show_roi_debug(frame_bgr, clf):
    roi = vision.default_roi(frame_bgr.shape)
    x, y, w, h = roi

    patch = frame_bgr[y:y+h, x:x+w].copy()
    ph, pw = patch.shape[:2]

    # shade the 15% border ring that classify_colour samples
    border = int(0.15 * min(ph, pw))
    overlay = patch.copy()
    cv2.rectangle(overlay, (0, 0), (pw, border), (0, 255, 255), -1)
    cv2.rectangle(overlay, (0, ph - border), (pw, ph), (0, 255, 255), -1)
    cv2.rectangle(overlay, (0, 0), (border, ph), (0, 255, 255), -1)
    cv2.rectangle(overlay, (pw - border, 0), (pw, ph), (0, 255, 255), -1)
    patch = cv2.addWeighted(patch, 0.65, overlay, 0.35, 0)
    colour, (H, S, V) = vision.classify_colour(frame_bgr, roi)
    label = f"HSV=({H:.1f},{S:.1f},{V:.1f})  classical colour: {colour}"
    cv2.rectangle(patch, (0, 0), (pw, 30), (30, 30, 30), -1)
    cv2.putText(patch, label, (8, 22), FONT, 0.55, (255, 255, 255), 1)
    cv2.namedWindow("DEBUG: ROI (what samplers see)", cv2.WINDOW_NORMAL)
    cv2.imshow("DEBUG: ROI (what samplers see)", patch)

    # the 28x28 image the LeNet fallback sees
    x_tensor = clf.digit_clf._preprocess(frame_bgr[y:y+h, x:x+w])
    if x_tensor is None:
        blank = np.zeros((310, 280, 3), dtype=np.uint8)
        cv2.putText(blank, "fallback: no contour",
                    (20, 140), FONT, 0.6, (0, 0, 255), 2)
        cv2.putText(blank, "(primary VLM may still answer)",
                    (20, 175), FONT, 0.5, (200, 200, 200), 1)
        cv2.namedWindow("DEBUG: 28x28 (classical fallback input)", cv2.WINDOW_NORMAL)
        cv2.imshow("DEBUG: 28x28 (classical fallback input)", blank)
        return
    img28 = x_tensor[0, 0]
    img28 = (img28 * vision.MNIST_STD) + vision.MNIST_MEAN
    img28 = np.clip(img28 * 255.0, 0, 255).astype(np.uint8)
    big = cv2.resize(img28, (280, 280), interpolation=cv2.INTER_NEAREST)
    big_bgr = cv2.cvtColor(big, cv2.COLOR_GRAY2BGR)
    disp = np.zeros((310, 280, 3), dtype=np.uint8)
    disp[:280] = big_bgr
    cv2.putText(disp, "classical fallback input (28x28)",
                (8, 300), FONT, 0.5, (255, 255, 255), 1)
    cv2.namedWindow("DEBUG: 28x28 (classical fallback input)", cv2.WINDOW_NORMAL)
    cv2.imshow("DEBUG: 28x28 (classical fallback input)", disp)


def do_inference(frame, clf, tport, store):
    """Classify the frame, query the board (or mock) and update store."""
    r = clf.classify_frame(frame)
    colour, digit = r["colour"], r["digit"]
    store["capture"] = {
        "colour": colour, "digit": digit,
        "confidence": r["confidence"], "hsv": r["hsv"],
        "backend": r.get("backend", "fallback"),
    }
    if not (colour and digit):
        store["result"] = InferResult(
            detected_colour=colour or "?", detected_digit=digit or "?",
            ok=False, note="colour or digit not recognised")
        return
    try:
        ctx_idx = hv.NAME_TO_CTX[colour]
        evt_idx = hv.NAME_TO_EVT[digit]
        ctx_hv  = hv.color_to_hv(colour)
        evt_hv  = hv.digit_to_hv(digit)
        store["result"] = tport.infer(ctx_hv, evt_hv, ctx_idx, evt_idx)
    except KeyError as e:
        store["result"] = InferResult(
            detected_colour=colour, detected_digit=digit,
            ok=False, note=f"not provisioned: {e}")


def run(args):
    if args.tcp_host:
        tport = make_transport("tcp", host=args.tcp_host, port=args.tcp_port)
        transport_label = f"TCP {args.tcp_host}:{args.tcp_port}"
    else:
        tport = make_transport("mock")
        transport_label = "MOCK (no FPGA)"

    try:
        clf = vision.FrameClassifier(
            args.onnx,
            qwen_model_path=(args.qwen_path or None),
            qwen_device=args.qwen_device,
            min_confidence=args.min_confidence,
        )
    except FileNotFoundError as e:
        sys.exit(f"[FATAL] {e}")

    # load Qwen now so the first capture doesn't wait for it
    if clf.qwen is not None:
        print("[startup] pre-loading vision model ...")
        clf.qwen._lazy_load()
        if clf.qwen._load_error is not None:
            print(f"[startup] VLM failed to load: {clf.qwen._load_error}")
            print(f"[startup] demo will run fallback-only")

    cap = cv2.VideoCapture(args.cam, cv2.CAP_ANY)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    if not cap.isOpened():
        sys.exit(f"Could not open camera {args.cam}")

    mode = "continuous" if args.continuous else "triggered"
    primary = "vlm" if args.qwen_path else "fallback-only"
    print(f"HippocamFLEX MNIST demo")
    print(f"  transport  : {transport_label}")
    print(f"  onnx       : {args.onnx}")
    print(f"  vlm  path  : {args.qwen_path or '(disabled)'}")
    print(f"  primary    : {primary}")
    print(f"  mode       : {mode}")
    print(f"  SPACE/ENTER -> capture + open debug windows")
    print(f"  Q/ESC       -> quit")

    store = {"capture": None, "result": None}

    # continuous mode only
    smoother = vision.Smoother(window=args.window, min_votes=args.min_votes)
    last_query = (None, None)
    last_query_t = 0.0
    cooldown_s = 0.8

    while True:
        ok, raw_frame = cap.read()
        if not ok:
            print("frame grab failed"); break
        display_frame = raw_frame if args.no_flip else cv2.flip(raw_frame, 1)
        roi = vision.default_roi(display_frame.shape)
        vision.draw_guide(display_frame, roi)

        # continuous mode classifies every frame; triggered mode only shows
        # the live HSV so Qwen doesn't run on every frame
        if mode == "continuous":
            r = clf.classify_frame(raw_frame)
            overlay_status(display_frame, r["colour"], r["digit"],
                           r["hsv"], r["confidence"], show_live=True,
                           backend=r.get("backend"))
            label = (r["colour"], r["digit"]) if (r["colour"] and r["digit"]) else None
            stable = smoother.update(label)
            now = time.time()
            if stable and (stable != last_query or now - last_query_t > cooldown_s):
                do_inference(raw_frame, clf, tport, store)
                last_query = stable
                last_query_t = now
        else:
            _, live_hsv = vision.classify_colour(raw_frame, roi)
            overlay_status(display_frame, None, None, live_hsv,
                           0.0, show_live=False)

        if store["capture"]:
            ci = store["capture"]
            overlay_capture_result(display_frame, ci["colour"],
                                   ci["digit"], ci["confidence"],
                                   backend=ci.get("backend"))
        if store["result"]:
            overlay_prediction(display_frame, store["result"])

        overlay_prompt(display_frame, mode)
        overlay_banner(display_frame, f"transport: {transport_label}")
        cv2.imshow("HippocamFLEX MNIST demo", display_frame)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if mode == "triggered" and key in (ord(" "), 13, 10):
            print("\n[CAPTURE]")
            do_inference(raw_frame, clf, tport, store)
            ci = store["capture"]
            print(f"  detected  : colour={ci['colour']}  "
                  f"digit={ci['digit']}  "
                  f"conf={ci['confidence']*100:.1f}%  "
                  f"via={ci.get('backend','fallback')}")
            res = store["result"]
            if res and res.ok:
                print(f"  prediction: VP={res.present}  VF1={res.future1}  "
                      f"VF2={res.future2}")
            else:
                print(f"  prediction: (no result) {res.note if res else ''}")
            show_roi_debug(raw_frame, clf)
        if mode == "continuous" and key == ord(" ") and last_query != (None, None):
            last_query_t = 0

    cap.release()
    cv2.destroyAllWindows()


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--cam", type=int, default=0)
    p.add_argument("--mock", action="store_true")
    p.add_argument("--tcp-host", type=str, default=None)
    p.add_argument("--tcp-port", type=int, default=9000)
    p.add_argument("--onnx", type=str, default="outputs/mnist_lenet.onnx",
                   help="fallback CNN weights (required)")
    p.add_argument("--qwen-path", type=str,
                   default="qwen_model/models/qwen2.5-vl-3b",
                   help="path to downloaded VLM weights. "
                        "Set to empty string to disable the VLM and run "
                        "fallback-only: --qwen-path ''")
    p.add_argument("--qwen-device", type=str, default="auto",
                   choices=["auto", "cuda", "cpu"])
    p.add_argument("--min-confidence", type=float, default=0.40)
    p.add_argument("--continuous", action="store_true")
    p.add_argument("--no-flip", action="store_true",
                   help="don't mirror the display preview")
    p.add_argument("--window", type=int, default=8)
    p.add_argument("--min-votes", type=int, default=5)
    return p.parse_args()


if __name__ == "__main__":
    run(parse_args())
