import os
import sys
from pathlib import Path


def configure_local_gstreamer():
    base = Path.home() / "Desktop" / "gst_local" / "extract"
    plugin_dirs = [
        base / "plugins_bad" / "usr" / "lib" / "x86_64-linux-gnu" / "gstreamer-1.0",
        base / "libav" / "usr" / "lib" / "x86_64-linux-gnu" / "gstreamer-1.0",
    ]
    helper_lib_dir = base / "bad" / "usr" / "lib" / "x86_64-linux-gnu"

    existing_plugins = [str(path) for path in plugin_dirs if path.is_dir()]
    if existing_plugins:
        current = os.environ.get("GST_PLUGIN_PATH", "")
        parts = existing_plugins + ([current] if current else [])
        os.environ["GST_PLUGIN_PATH"] = ":".join(parts)
        os.environ.setdefault("GST_REGISTRY", "/tmp/gst-local-registry.bin")

    if helper_lib_dir.is_dir():
        current = os.environ.get("LD_LIBRARY_PATH", "")
        parts = [str(helper_lib_dir)] + ([current] if current else [])
        os.environ["LD_LIBRARY_PATH"] = ":".join(parts)


configure_local_gstreamer()

import cv2


def has_gstreamer_support():
    info = cv2.getBuildInformation()
    for line in info.splitlines():
        if "GStreamer" in line:
            return line.strip().endswith("YES") or "YES (" in line
    return False


if not has_gstreamer_support():
    if os.environ.get("PYTHONNOUSERSITE") != "1":
        new_env = os.environ.copy()
        new_env["PYTHONNOUSERSITE"] = "1"
        os.execvpe(sys.executable, [sys.executable, __file__, *sys.argv[1:]], new_env)

    print("OpenCV was loaded without GStreamer support.")
    print("Loaded from:", cv2.__file__)
    sys.exit(1)

PORT = 5000

PIPELINE = (
    f'udpsrc port={PORT} ! '
    'h264parse ! '
    'avdec_h264 ! '
    'videoconvert ! '
    'appsink max-buffers=1 drop=true sync=false'
)

cap = cv2.VideoCapture(PIPELINE, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Failed to open UDP H.264 stream.")
    print("Close ffplay first if it is still using udp://@:5000.")
    sys.exit(1)

print("Press ESC to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    h, s, v = cv2.split(hsv)
    cv2.imshow("Hue", h)
    cv2.imshow("Saturation", s)
    cv2.imshow("Value", v)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
