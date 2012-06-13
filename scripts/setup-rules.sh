#!/bin/bash

# USAGE: setup-rules.sh <username>

sudo adduser $1 video
if [ ! -f /etc/udev/rules.d/51-kinect.rules ]
then
cat << EOF >> /etc/udev/rules.d/51-kinect.rules
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE="0666", GROUP="video"
# ATTR{product}=="Xbox NUI Audio"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE="0666", GROUP="video"
# ATTR{product}=="Xbox NUI Camera"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666", GROUP="video"
EOF
else
echo "udev rules already exist"
fi

if [ ! -f /etc/ld.so.conf.d/usr-local-libs.conf ]
then
sudo cat << EOF >> /etc/ld.so.conf.d/usr-local-libs.conf
/usr/local/lib64
/usr/local/lib
EOF
else
echo "usr-local-libs.conf already exists"
fi
