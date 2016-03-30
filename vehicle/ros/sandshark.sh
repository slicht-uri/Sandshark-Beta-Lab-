#!/bin/bash

#If you want to not have to type the sandshark and operator IP addresses every time this script is invoked,
#change these to reflect your setup
sandsharkIP=192.168.1.100
operatorIP=192.168.1.200

tempd=$(mktemp)
trap "rm -f -- '${tempd}'" EXIT
cat <<-EOF > ${tempd}

# setup the bash prompt and export all necessary paths
if [ "$1" == "sim" ]; then
  export ROS_MASTER_URI=http://127.0.0.1:11311
  export PS1='\[\033[01;36m\]sandshark-simulator\[\033[00m\]:\w\[\033[00m\]\$ '

  export PYTHONPATH=$(echo $PYTHONPATH | perl -pe "s[/data/app/bluefin/opt/sandshark/lib/python2.7/dist-packages][]g;" >/dev/null 2>/dev/null) 
  export ROS_PACKAGE_PATH=$(echo $ROS_PACKAGE_PATH | perl -pe "s[/data/app/bluefin/opt/sandshark][]g;" >/dev/null 2>/dev/null)
  export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | perl -pe "s[/data/app/bluefin/opt/sandshark/lib/][]g;" >/dev/null 2>/dev/null)

  export PYTHONPATH=$PYTHONPATH:/data/app/bluefin/opt/sandshark_sim/lib/python2.7/dist-packages
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/data/app/bluefin/opt/sandshark_sim
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/data/app/bluefin/opt/sandshark_sim/lib/

else

  export ROS_MASTER_URI=http://${1-$sandsharkIP}:11311
  export ROS_IP=${2-$operatorIP}
  export PS1='\[\033[01;33m\]sandshark-${1-$sandsharkIP}\[\033[00m\]:\w\[\033[00m\]\$ '

  export PYTHONPATH=$(echo $PYTHONPATH | perl -pe "s[/data/app/bluefin/opt/sandshark_sim/lib/python2.7/dist-packages][]g;" >/dev/null 2>/dev/null)
  export ROS_PACKAGE_PATH=$(echo $ROS_PACKAGE_PATH | perl -pe "s[/data/app/bluefin/opt/sandshark_sim)][]g;" >/dev/null 2>/dev/null)
  export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | perl -pe "s[/data/app/bluefin/opt/sandshark_sim/lib/][]g;" >/dev/null 2>/dev/null)

  export PYTHONPATH=$PYTHONPATH:/data/app/bluefin/opt/sandshark/lib/python2.7/dist-packages
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/data/app/bluefin/opt/sandshark
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/data/app/bluefin/opt/sandshark/lib/
fi

EOF

${SHELL} --rcfile ${tempd}
rm -f -- "${tempd}"
trap - EXIT

