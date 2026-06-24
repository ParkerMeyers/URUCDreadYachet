#!/bin/bash
echo "Connecting to ROV Dual Video Feeds..."

# Trap command ensures that if you press Ctrl+C, it cleanly kills both video windows
trap "killall gst-launch-1.0" EXIT

# Open Window 1 (Port 5600)
gst-launch-1.0 -v udpsrc port=5600 caps="application/x-rtp, media=video, clock-rate=90000, encoding-name=H264" ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false &

# Open Window 2 (Port 5601)
gst-launch-1.0 -v udpsrc port=5601 caps="application/x-rtp, media=video, clock-rate=90000, encoding-name=H264" ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false &

# Wait endlessly while the video plays
wait
