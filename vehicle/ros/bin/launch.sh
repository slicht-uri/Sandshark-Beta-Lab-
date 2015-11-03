#!/system/bin/sh

#Hack to get cam1 vdio enabled
mount -t debugfs none /d
echo 1 > /d/regulator/8921_lvs4/enable
echo 1 > /d/regulator/8921_lvs5/enable

#Enable FPGA GPIO - starts at 400
modprobe titan-fpga

#Enable RS485 ttys
modprobe max310x

#Get the Ethernet up
echo 407 > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio407/direction
echo 1 > /sys/class/gpio/gpio407/value

#Turn on payload power
echo 420 > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio420/direction
echo 1 > /sys/class/gpio/gpio420/value

modprobe smsc75xx
sleep 2
busybox ifconfig eth0 up
busybox ifconfig eth0 10.0.7.65

busybox ifconfig eth0:0 up
busybox ifconfig eth0:0 192.168.23.120

wl scansuppress 1

sleep 5

wipa=`cat /data/app/bluefin/bin/ip_address.txt`
busybox ifconfig wlan0 $wipa

# Wait to see if the SD card mounts for logging
for i in $(seq 1 15); do
    grep -q sdcard /proc/mounts && break
    sleep 1
done

. /etc/mkshrc

export ROS_IP=$wipa

ulimit -c unlimited
echo "/data/app/core-%e.%p" > /proc/sys/kernel/core_pattern

echo 5 > /proc/cpu/alignment

am startservice com.bluefinrobotics.bluefingps/com.bluefinrobotics.bluefingps.BluefinAdaptGpsService

sleep 5

#export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/data/app/bluefin/opt/ros_xc/lib
#export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/data/app/bluefin/opt/ros_xc/lib

source /data/app/bluefin/opt/ros_xc/share/sandshark_common/launch/runtimeEnv.bash

killall python rosout sandshark_mavlink_node sandshark_gps_node sandshark_motion_node sandshark_navigation_node

taskset 0x01 roslaunch /data/app/bluefin/opt/ros_xc/share/sandshark_common/launch/catkin_all.launch&


sleep 2

rosbag record -a -x "/health/(.*)|/motion/(.*)" --split --duration=10m -o /mnt/sdcard/bluefin
