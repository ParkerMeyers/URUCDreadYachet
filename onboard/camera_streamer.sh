#!/bin/bash
# Stream dual ROV cameras as H.264 RTP/UDP to the topside laptop.
# The topside UI listens on UDP ports 5600 and 5601 (see topside/ROV_Cameras.sh).
#
# Usage (on Pi):
#   ROV_TOPSIDE_IP=192.168.2.50 ./onboard/camera_streamer.sh
#
# Overrides:
#   ROV_CAM1_DEVICE=/dev/video0
#   ROV_CAM2_DEVICE=/dev/video2
#   ROV_CAMERA1_PIPELINE='... full gst-launch pipeline ending with udpsink ...'
#   ROV_CAMERA2_PIPELINE='...'

set -euo pipefail

TOPSIDE_IP="${1:-${ROV_TOPSIDE_IP:-}}"
if [[ -z "${TOPSIDE_IP}" ]]; then
  echo "camera_streamer: ROV_TOPSIDE_IP / topside IP argument is required" >&2
  exit 1
fi

CAM1_DEV="${ROV_CAM1_DEVICE:-/dev/video0}"
CAM2_DEV="${ROV_CAM2_DEVICE:-/dev/video2}"
PORT1="${ROV_CAMERA1_PORT:-5600}"
PORT2="${ROV_CAMERA2_PORT:-5601}"

stop_existing() {
  pkill -f "uru_camera_gst_cam1" 2>/dev/null || true
  pkill -f "uru_camera_gst_cam2" 2>/dev/null || true
  sleep 0.2
}

default_send_pipeline() {
  local dev="$1"
  local port="$2"
  local tag="$3"
  echo "v4l2src device=${dev} ! video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=1200 key-int-max=30 ! video/x-h264,profile=baseline ! rtph264pay config-interval=1 pt=96 ! udpsink host=${TOPSIDE_IP} port=${port} sync=false async=false name=${tag}"
}

launch_camera() {
  local pipeline="$1"
  local log_tag="$2"
  echo "camera_streamer: starting ${log_tag} -> ${TOPSIDE_IP}"
  nohup gst-launch-1.0 -q ${pipeline} >"/tmp/uru_${log_tag}.log" 2>&1 < /dev/null &
}

stop_existing

if [[ -n "${ROV_CAMERA1_PIPELINE:-}" ]]; then
  launch_camera "${ROV_CAMERA1_PIPELINE}" "camera_gst_cam1"
else
  launch_camera "$(default_send_pipeline "${CAM1_DEV}" "${PORT1}" "uru_camera_gst_cam1")" "camera_gst_cam1"
fi

if [[ -n "${ROV_CAMERA2_PIPELINE:-}" ]]; then
  launch_camera "${ROV_CAMERA2_PIPELINE}" "camera_gst_cam2"
else
  launch_camera "$(default_send_pipeline "${CAM2_DEV}" "${PORT2}" "uru_camera_gst_cam2")" "camera_gst_cam2"
fi

echo "camera_streamer: streaming to ${TOPSIDE_IP}:${PORT1} and ${TOPSIDE_IP}:${PORT2}"
sleep 0.5
echo "camera_streamer: launched"
