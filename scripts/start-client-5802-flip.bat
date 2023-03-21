@echo off

:: This can be used to start listening to a GStreamer UDP (on port 5802), and display as well as record the footage to an mkv file
:: Do not change the port number, as it is one of the FMS-compatible UDP ports available to FRC teams. 
:: For convenience, creating a shortcut on the desktop to this script is recommended

gst-launch-1.0 -e udpsrc port=5802 ! "application/x-rtp, clock-rate=(int)90000, media=(string)video, encoding-name=VP8" ! rtpvp8depay ! tee name = t t. ! queue ! decodebin ! videoflip method=counterclockwise ! autovideosink t. ! queue ! matroskamux ! filesink location=%time:~0,2%_%time:~3,2%_%time:~6,2%__%date:~4,2%_%date:~7,2%_%date:~10,4%_5802.mkv
pause
:: If the script fails then wait for confirmation before closing