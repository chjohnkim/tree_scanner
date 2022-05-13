#!/bin/sh

echo "Init 2 Rostopic Pub"
rostopic pub /ping_usb_dev std_msgs/String "t" -1
rostopic pub /ping std_msgs/String "t" -r 125


