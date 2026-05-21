#!/usr/bin/env python3
# encoding: utf-8

# =============================================================================
#
# mic_live.py : live direction-of-arrival from ReSpeaker Mic Array V2.0 
#
# Written by Jonathan H. Connell, jconnell@alum.mit.edu
#
# -----------------------------------------------------------------------------
#
# Copyright 2026 Etaoin Systems
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

import sys, time
import usb.core
import usb.util
from mic_tuning import Tuning


# prints out speech flag and direction of sound (USB plug = 270 deg CCW)
# to obtain non-sudo USB permissions make file /etc/udev/rules.d/51-usb-device.rules with line:
# SUBSYSTEM=="usb", ATTRS{idVendor}=="2886", ATTRS{idProduct}=="0018", GROUP="plugdev", TAG+="uaccess"

class MicLive:

  def __init__(self):

    # connect to ReSpeaker via USB
    dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
    if dev:
      mic = Tuning(dev)
    else:
      rospy.loginfo("Could not open ReSpeaker 4 Mic Array on USB")
      rospy.signal_shutdown("No mic array")

    # possibly set speech detection level
    db = 15.0                                    # power-on default
    if len(sys.argv) > 1:
      try:
        db = float(sys.argv[1])   
      except:
        pass
    mic.set_vad_threshold(db)

    # main loop runs at 30 Hz (Ctrl-C to exit)
    print("Sound direction over 20 secs ...");
    v = 1
    for i in range(600):
      if not mic.is_voice():
        if v > 0:
          print("  -----")
        v = 0
      else:
        # direction +/- 180 deg CCW, 0 if no voice, USB @ -135 deg
        cord = -135
        dir = mic.direction + cord + 90
        if dir <= -180:
          dir += 360
        elif dir > 180:
          dir -= 360 
        print("  %3d degs" % (dir))
        v = 1
      time.sleep(0.033)
    print("Done")
   

# =========================================================================

# connect and start loop

if __name__ == "__main__":  
  m = MicLive()

  
