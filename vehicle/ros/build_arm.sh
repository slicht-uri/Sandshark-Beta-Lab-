#!/bin/bash

#See if sim executables are present and erase them
if [ ! -f devel/lib/sandshark_drivers/sandshark_motion_node ]; then
    rm -r devel
    rm -r build
fi

export PYTHONPATH=/opt/ros/indigo/lib/python2.7/dist-packages
export LD_LIBRARY_PATH=/opt/ros/indigo/lib:/opt/ros/indigo/lib/x86_64-linux-gnu:/opt/ros/indigo/lib/python2.7/dist-packages
export LIB=/usr/local/bluefin/adapt-sysroot/lib
export INCLUDE=/usr/local/bluefin/adapt-sysroot/usr/include
export ROS_DISTRO=hydro
export ROS_ROOT=/usr/local/bluefin/adapt-sysroot/ros_install_isolated/share/ros
export ROS_PACKAGE_PATH=/usr/local/bluefin/adapt-sysroot/ros_install_isolated/share
export ROS_ETC_DIR=/usr/local/bluefin/adapt-sysroot/ros_install_isolated/etc/ros
export CMAKE_PREFIX_PATH=/usr/local/bluefin/adapt-sysroot/ros_install_isolated

/opt/ros/indigo/bin/catkin_make install -DCMAKE_INSTALL_PREFIX=/data/app/bluefin/opt/ros_xc \
                                        -DCMAKE_TOOLCHAIN_FILE=$PWD/android.toolchain \
                                        -DIN_SIM=OFF \
                                        -DPYTHON_INCLUDE_DIR=/usr/local/bluefin/adapt-sysroot/usr/include/python2.7/ \
                                        -DPYTHON_LIBRARY=/usr/local/bluefin/adapt-sysroot/lib/libpython2.7.so \
                                        -DPYTHON_INCLUDE_DIR2=/usr/local/bluefin/adapt-sysroot/usr/include/python2.7/ \
                                        --make-args VERBOSE=1

if [ $# -gt 0 ]; then
    CUR_DIR=$PWD
    echo "cur dir"
    echo $CUR_DIR
    cd /data/app/bluefin/opt/
    tar -czf catkin_package.tgz ros_xc
    cd $CUR_DIR
    ssh root@$1 'rm -r /data/app/bluefin/opt'
    ssh root@$1 'mkdir /data/app/bluefin/opt'
    scp /data/app/bluefin/opt/catkin_package.tgz root@$1:/data/app/bluefin/opt/
    scp bin/launch.sh root@$1:/data/app/bluefin/bin
    ssh root@$1 'cd /data/app/bluefin/opt; rm -rf ros_xc 2> /dev/null; tar -xf catkin_package.tgz'
    scp sysfiles/mkshrc root@$1:/system/etc
    scp sysfiles/authorized_keys root@$1:/data/ssh/
    scp sysfiles/ip_address.txt root@$1:/data/app/bluefin/bin
    ssh root@$1 'chmod 600 /data/ssh/authorized_keys'
    scp sysfiles/wpa_supplicant.conf root@$1:/data/misc/wifi
else
    echo "Not Deploying, no IP address passed"
fi
