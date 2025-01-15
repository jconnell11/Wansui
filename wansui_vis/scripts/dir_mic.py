#!/usr/bin/env python3
# encoding: utf-8

# =============================================================================
#
# dir_mic.py : direction-of-arrival from ReSpeaker Mic Array V2.0
#
# Written by Jonathan H. Connell, jconnell@alum.mit.edu
#
# -----------------------------------------------------------------------------
#
# Copyright 2025 Etaoin Systems
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# 
# =============================================================================

import rospy
from std_msgs.msg import Int32
import usb.core
import usb.util
from mic_tuning import Tuning


# broadcasts whether sound heard and, if so, direction of arrival 
# to obtain non-sudo USB permissions make file /etc/udev/rules.d/51-usb-device.rules with line:
# SUBSYSTEM=="usb", ATTRS{idVendor}=="2886", ATTRS{idProduct}=="0018", GROUP="plugdev", TAG+="uaccess"

class DirMic:

  def __init__(self):

    # create a ROS node which publishes "sound_dir" messages 
    rospy.init_node('dir_mic', anonymous=True)
    dir_pub = rospy.Publisher('sound_dir', Int32, queue_size=1)

    # connect to ReSpeaker via USB
    dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
    if dev:
      mic = Tuning(dev)
    else:
      rospy.loginfo("Could not open ReSpeaker 4 Mic Array on USB")
      rospy.signal_shutdown("No mic array")

    # main loop runs at 30 Hz
    r = rospy.Rate(30)
    while not rospy.is_shutdown():

      # direction 1-360 deg CCW, 0 if no voice, cord @ 180 deg
      dir = mic.direction - 90
      if dir <= 0:
        dir += 360
      if not mic.is_voice():
        dir = 0

      # send out current estimate
      dir_pub.publish(Int32(dir))      
      r.sleep()
    

# =========================================================================

# make node start broadcasting

if __name__ == "__main__":
  try:
    d = DirMic()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
  
