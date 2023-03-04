gst-launch-1.0 -e udpsrc port=5801 ! "application/x-rtp, clock-rate=(int)90000, media=(string)video, encoding-name=VP8" ! rtpvp8depay ! tee name = t t. ! queue ! decodebin ! videoflip method=rotate-180 ! autovideosink t. ! queue ! matroskamux ! filesink location=/videos/%time:~0,2%_%time:~3,2%_%time:~6,2%__%date:~4,2%_%date:~7,2%_%date:~10,4%_5801.mkv

Read-Host -Prompt "Press Enter to exit"
