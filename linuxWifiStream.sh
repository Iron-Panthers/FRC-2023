#! /usr/bin/bash

gst-launch-1.0 -e udpsrc port=5801 ! "application/x-rtp, clock-rate=(int)90000, media=(string)video, encoding-name=VP8" ! rtpvp8depay ! tee name = t t. ! queue ! decodebin ! videoflip method=rotate-180 ! autovideosink 