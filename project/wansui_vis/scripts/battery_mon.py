#!/usr/bin/env python3
# encoding: utf-8

# =============================================================================
#
# battery_mon.py : broadcasts smoothed estimate of battery voltage
#
# Written by Jonathan H. Connell, jconnell@alum.mit.edu
#
# -----------------------------------------------------------------------------
#
# Copyright 2024 Etaoin Systems
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
from std_msgs.msg import Float32
from jetauto_sdk import hiwonder_servo_controller


# broadcasts smoothed estimate of battery voltage

class BatteryMonitor:

  # set up ROS parameters and start main loop
  def __init__(self):

    # create a ROS node which publishes "voltage" messages 
    rospy.init_node('battery_mon', anonymous=True)
    volt_pub = rospy.Publisher('voltage', Float32, queue_size=1)

    # read battery voltage from digital servo
    servos = hiwonder_servo_controller.HiwonderServoController('/dev/ttyTHS1', 115200)
    voltage = 12.0

    # main loop runs at 1 Hz
    r = rospy.Rate(1)
    while not rospy.is_shutdown():

      # if not crazy add sample to IIR filter 
      v = servos.get_servo_vin(5) / 1000.0
      if v > 7.5 and v < 13.0:
        voltage += 0.2 * (v - voltage)

      # send out current estimate
      volt_pub.publish(Float32(voltage))      
      r.sleep()
    

# =========================================================================

# make node start broadcasting

if __name__ == "__main__":
  try:
    m = BatteryMonitor()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
  
