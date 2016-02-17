#!/system/bin/sh
# Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Code Aurora nor
#       the names of its contributors may be used to endorse or promote
#       products derived from this software without specific prior written
#       permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

LOG_TAG="qcom-bluetooth"
LOG_NAME="${0}:"

hciattach_pid=""

loge ()
{
  /system/bin/log -t $LOG_TAG -p e "$LOG_NAME $@"
}

logi ()
{
  /system/bin/log -t $LOG_TAG -p i "$LOG_NAME $@"
}

failed ()
{
  loge "$1: exit code $2"
  exit $2
}

start_hciattach ()
{
  logi "start_hciattach"
  brcm_patchram_plus --enable_hci --no2bytes --enable_lpm --tosleep 50000 --baudrate 3000000 --use_baudrate_for_download --patchram /system/etc/BCM4330.hcd /dev/ttyHS1 &
  hciattach_pid=$!
  logi "start_hciattach: pid = $hciattach_pid"
}

kill_hciattach ()
{
  logi "kill_hciattach: pid = $hciattach_pid"
  ## careful not to kill zero or null!
  kill -TERM $hciattach_pid
  # this shell does not exit now -- wait returns for normal exit
}

cd /system/bin

trap "kill_hciattach" TERM INT

config_bt ()
{
  baseband=`getprop ro.baseband`
  target=`getprop ro.board.platform`
  soc_hwid=`cat /sys/devices/system/soc/soc0/id`
  btsoc=`getprop qcom.bluetooth.soc`

  case $baseband in
    "apq")
        setprop ro.qualcomm.bluetooth.opp true
        setprop ro.qualcomm.bluetooth.ftp true
        setprop ro.qualcomm.bluetooth.nap false
        setprop ro.qualcomm.bluetooth.sap false
        setprop ro.qualcomm.bluetooth.dun false
        # For MPQ as baseband is same for both
        case $soc_hwid in
          "130")
              setprop ro.qualcomm.bluetooth.hsp true
              setprop ro.qualcomm.bluetooth.hfp true
              setprop ro.qualcomm.bluetooth.pbap false
              setprop ro.qualcomm.bluetooth.map false
              ;;
          *)
              setprop ro.qualcomm.bluetooth.hsp false
              setprop ro.qualcomm.bluetooth.hfp false
              setprop ro.qualcomm.bluetooth.pbap true
              setprop ro.qualcomm.bluetooth.map true
              ;;
        esac
        ;;
    "mdm" | "svlte2a" | "svlte1" | "csfb")
        setprop ro.qualcomm.bluetooth.opp true
        setprop ro.qualcomm.bluetooth.hfp true
        setprop ro.qualcomm.bluetooth.hsp true
        setprop ro.qualcomm.bluetooth.pbap true
        setprop ro.qualcomm.bluetooth.ftp true
        setprop ro.qualcomm.bluetooth.map true
        setprop ro.qualcomm.bluetooth.nap true
        setprop ro.qualcomm.bluetooth.sap true
        setprop ro.qualcomm.bluetooth.dun false
        ;;
    "msm")
        setprop ro.qualcomm.bluetooth.opp true
        setprop ro.qualcomm.bluetooth.hfp true
        setprop ro.qualcomm.bluetooth.hsp true
        setprop ro.qualcomm.bluetooth.pbap true
        setprop ro.qualcomm.bluetooth.ftp true
        setprop ro.qualcomm.bluetooth.nap true
        setprop ro.qualcomm.bluetooth.sap true
        setprop ro.qualcomm.bluetooth.dun true
        case $btsoc in
          "ath3k")
              setprop ro.qualcomm.bluetooth.map false
              ;;
          *)
              setprop ro.qualcomm.bluetooth.map true
              ;;
        esac
        ;;
    *)
        setprop ro.qualcomm.bluetooth.opp true
        setprop ro.qualcomm.bluetooth.hfp true
        setprop ro.qualcomm.bluetooth.hsp true
        setprop ro.qualcomm.bluetooth.pbap true
        setprop ro.qualcomm.bluetooth.ftp true
        setprop ro.qualcomm.bluetooth.map true
        setprop ro.qualcomm.bluetooth.nap true
        setprop ro.qualcomm.bluetooth.sap true
        setprop ro.qualcomm.bluetooth.dun true
        ;;
  esac

  #Enable Bluetooth Profiles specific to target Dynamically
  case $target in
    "msm8960")
       if [ "$btsoc" != "ath3k" ] && [ "$socid" != "130" ]
       then
           setprop ro.bluetooth.hfp.ver 1.6
           #setprop ro.qualcomm.bt.hci_transport smd
       fi
       ;;
    *)
       ;;
  esac

}

config_bt

start_hciattach

wait $hciattach_pid

exit 0
