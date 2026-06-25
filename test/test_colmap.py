"""Self-check for the topside COLMAP recorder: no-distortion fit + single-folder save.

Run:  python test/test_colmap.py
"""
import os
import sys
import shutil

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import rov_ui


def test_fit_no_distortion():
    # 1280x960 (4:3) -> capped to 720 height, aspect ratio preserved.
    big = np.zeros((960, 1280, 3), dtype=np.uint8)
    out = rov_ui._colmap_fit(big)
    assert out.shape[0] == 720, out.shape
    assert abs((out.shape[1] / out.shape[0]) - (1280 / 960)) < 0.01, out.shape
    # Already <=720p stays untouched (never upscale, never distort).
    small = np.zeros((480, 640, 3), dtype=np.uint8)
    assert rov_ui._colmap_fit(small).shape == small.shape


def test_save_single_folder():
    import cv2
    os.makedirs(rov_ui.COLMAP_STAGE, exist_ok=True)
    for f in os.listdir(rov_ui.COLMAP_STAGE):
        os.remove(os.path.join(rov_ui.COLMAP_STAGE, f))
    blank = np.zeros((480, 640, 3), dtype=np.uint8)
    for i in range(1, 4):
        cv2.imwrite(os.path.join(rov_ui.COLMAP_STAGE, f"frame_{i:06d}.jpg"), blank)

    rov_ui._colmap["running"] = False
    rov_ui._colmap["thread"] = None
    d = rov_ui.app.test_client().post("/api/colmap/save").get_json()
    assert d["ok"] and d["count"] == 3, d
    saved = sorted(os.listdir(d["folder"]))
    assert saved == [f"frame_{i:06d}.jpg" for i in range(1, 4)], saved
    assert not os.listdir(rov_ui.COLMAP_STAGE), "staging not drained"
    shutil.rmtree(d["folder"])


if __name__ == "__main__":
    test_fit_no_distortion()
    test_save_single_folder()
    print("ok")
