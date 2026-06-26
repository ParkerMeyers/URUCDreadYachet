#!/usr/bin/env python3
"""
crab_detector.py — Topside European Green Crab detector for MATE Task 2.1.

Receives the H.264 RTP/UDP stream from Camera 1 (port 5600), runs each frame
through crab_model.onnx (YOLO11m) with onnxruntime, draws a bounding box around
every european-green-crab detected above 0.5 confidence, and shows the running
count on the video display.

Only european-green-crab is boxed/counted. Native rock crabs and jonah crabs are
ignored entirely (per the task: they must NOT be boxed nor included in the count).

Controls (while the video window is focused):
    s  -> save the current annotated frame as a PNG
    q  -> quit

No ROS. OpenCV + onnxruntime only.
"""

import os
import time
import argparse

import numpy as np
import cv2
import onnxruntime as ort

# --- Model / class config (matches crab_classifier_node.py) -------------------
CLASS_NAMES = ['european-green-crab', 'jonah-crab', 'native-rock-crab']
GREEN_CRAB_ID = 0                  # index of european-green-crab in CLASS_NAMES
INPUT_SIZE = 512                   # model expects 512x512

# --- Detection tuning ---------------------------------------------------------
CONF_THRESHOLD = 0.6               # count detections at/above this confidence (was 0.5 task min)
NMS_THRESHOLD = 0.45               # IoU threshold for de-duplicating boxes

# --- Stream config ------------------------------------------------------------
DEFAULT_PORT = 5600                # Camera 1 (topside receive port)


def build_gst_pipeline(port: int) -> str:
    """GStreamer receive pipeline matching current_camera.sh (x264 + rtph264pay, pt=96)."""
    return (
        f"udpsrc port={port} "
        "caps=\"application/x-rtp, media=video, clock-rate=90000, "
        "encoding-name=H264, payload=96\" ! "
        "rtph264depay ! h264parse ! avdec_h264 ! "
        "videoconvert ! appsink sync=false drop=true max-buffers=1"
    )


def preprocess(frame: np.ndarray) -> np.ndarray:
    """Identical preprocessing to crab_classifier_node.py.

    Resize -> BGR2RGB -> /255.0 -> HWC->CHW -> add batch dim -> (1, 3, 512, 512).
    """
    img = cv2.resize(frame, (INPUT_SIZE, INPUT_SIZE))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))          # HWC -> CHW
    return np.expand_dims(img, axis=0)          # -> (1, 3, 512, 512)


def postprocess(output, frame_w: int, frame_h: int):
    """Decode YOLO11 ONNX output into european-green-crab boxes in frame pixels.

    Output shape: (1, 7, N). Each column is [cx, cy, w, h, c0, c1, c2] where
    cx/cy/w/h are in the 512x512 input space. We keep only boxes whose top class
    is european-green-crab with confidence >= CONF_THRESHOLD, then run NMS so each
    physical crab yields exactly one box (the raw model emits many overlaps).

    Returns a list of (x1, y1, x2, y2, confidence) in original-frame pixels.
    """
    preds = output[0][0].T            # (N, 7)

    # Scale factors from the 512x512 input back to the original frame.
    scale_x = frame_w / INPUT_SIZE
    scale_y = frame_h / INPUT_SIZE

    boxes = []        # [x, y, w, h] in frame pixels, for cv2.dnn.NMSBoxes
    scores = []

    for pred in preds:
        class_scores = pred[4:]
        class_id = int(np.argmax(class_scores))
        confidence = float(class_scores[class_id])

        # Only european-green-crab above threshold; everything else is ignored.
        if class_id != GREEN_CRAB_ID or confidence < CONF_THRESHOLD:
            continue

        cx, cy, w, h = pred[0], pred[1], pred[2], pred[3]
        x1 = (cx - w / 2.0) * scale_x
        y1 = (cy - h / 2.0) * scale_y
        bw = w * scale_x
        bh = h * scale_y

        boxes.append([int(x1), int(y1), int(bw), int(bh)])
        scores.append(confidence)

    if not boxes:
        return []

    keep = cv2.dnn.NMSBoxes(boxes, scores, CONF_THRESHOLD, NMS_THRESHOLD)
    if len(keep) == 0:
        return []

    detections = []
    for i in np.array(keep).flatten():
        x, y, w, h = boxes[i]
        detections.append((x, y, x + w, y + h, scores[i]))
    return detections


def draw_overlay(frame: np.ndarray, detections):
    """Draw a box + label on each green crab and the total count on the frame."""
    for (x1, y1, x2, y2, conf) in detections:
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"green-crab {conf:.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw, y1), (0, 255, 0), -1)
        cv2.putText(frame, label, (x1, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

    count_text = f"European Green Crabs: {len(detections)}"
    cv2.rectangle(frame, (0, 0), (360, 36), (0, 0, 0), -1)
    cv2.putText(frame, count_text, (8, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
    return frame


def main():
    parser = argparse.ArgumentParser(description="MATE Task 2.1 green-crab detector (topside).")
    parser.add_argument("--model", default=os.path.join(os.path.dirname(__file__), "crab_model.onnx"),
                        help="Path to crab_model.onnx")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                        help="UDP port to receive Camera 1 (default 5600)")
    parser.add_argument("--outdir", default=os.path.dirname(__file__),
                        help="Directory to save PNG captures")
    args = parser.parse_args()

    # --- Load model -----------------------------------------------------------
    print(f"[crab_detector] Loading model: {args.model}")
    session = ort.InferenceSession(args.model, providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name

    # --- Open stream ----------------------------------------------------------
    pipeline = build_gst_pipeline(args.port)
    print(f"[crab_detector] Opening stream on UDP port {args.port}")
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("[crab_detector] ERROR: could not open GStreamer pipeline.\n"
              "  - Is the ROV streaming (current_camera.sh running)?\n"
              "  - Was OpenCV built with GStreamer support? "
              "(cv2.getBuildInformation() should list GStreamer: YES)")
        return

    win = "Crab Detector - Camera 1"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    print("[crab_detector] Running. Press 's' to save a PNG, 'q' to quit.")

    saved = 0
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            # Transient on UDP; keep trying rather than dying.
            cv2.waitKey(1)
            continue

        h, w = frame.shape[:2]
        tensor = preprocess(frame)
        outputs = session.run(None, {input_name: tensor})
        detections = postprocess(outputs, w, h)
        annotated = draw_overlay(frame, detections)

        cv2.imshow(win, annotated)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('s'):
            fname = os.path.join(
                args.outdir, f"crab_capture_{time.strftime('%Y%m%d_%H%M%S')}.png")
            cv2.imwrite(fname, annotated)
            saved += 1
            print(f"[crab_detector] Saved {fname} "
                  f"({len(detections)} green crab(s) in frame)")

    cap.release()
    cv2.destroyAllWindows()
    print(f"[crab_detector] Done. {saved} capture(s) saved.")


if __name__ == "__main__":
    main()
