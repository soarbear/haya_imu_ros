#!/bin/bash

echo "delete remap the device serial port(ttyACM*) of haya_imu"
echo "sudo rm /etc/udev/rules.d/haya_imu.rules"
sudo rm /etc/udev/rules.d/haya_imu.rules
echo ""
echo "restarting udev..."
echo ""
sudo service udev reload
sudo service udev restart
echo "done"
