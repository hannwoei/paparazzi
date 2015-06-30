#!/bin/bash
gst-launch-0.10 udpsrc uri=udp://0.0.0.0:5000  caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)JPEG, payload=(int)26" ! rtpjpegdepay ! jpegdec ! ffmpegcolorspace ! xvimagesink

