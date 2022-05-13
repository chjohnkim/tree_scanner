#!/bin/sh

echo "Init 1 Syncbox Control"
echo "Giving USB permissions..."
sudo chmod 777 /dev/ttyXRUSB0  /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyACM0
echo "Set USB buffer size for USB camera..."
sudo sh -c 'echo 2048 > /sys/module/usbcore/parameters/usbfs_memory_mb'
echo "Rosrun syncbox_control udev_ping_umass.py..."
rosrun syncbox_control udev_ping_umass.py

#read PERSON
#echo "Hello, $PERSON"
