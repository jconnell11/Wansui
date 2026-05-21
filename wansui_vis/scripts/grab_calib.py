#!/usr/bin/env python3
# encoding: utf-8

# =============================================================================
#
# grab_calib.py : fine calibration of JetAuto hand position
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

import os, sys, tty, termios, math, socket, yaml
import rospy

from hiwonder_servo_msgs.msg import CommandDuration as JtCmd


# -----------------------------------------------------------------------------

# controller for multi-step calibration of arm servo angles

class GrabCalib:

  # constructor initializes some items
  def __init__(self):

    # enroll with ROS as a node
    rospy.init_node('grab_calib', anonymous=True)
    
    # -------------- actuators ----------------- 

    # set up to communicate with arm servos (b, s, e, w, g, p, t)
    self.pub = [
      rospy.Publisher( "joint1_controller/command_duration", JtCmd, queue_size=1),
      rospy.Publisher( "joint2_controller/command_duration", JtCmd, queue_size=1),
      rospy.Publisher( "joint3_controller/command_duration", JtCmd, queue_size=1),
      rospy.Publisher( "joint4_controller/command_duration", JtCmd, queue_size=1),
      rospy.Publisher("r_joint_controller/command_duration", JtCmd, queue_size=1),
      rospy.Publisher("p_joint_controller/command_duration", JtCmd, queue_size=1),
      rospy.Publisher("t_joint_controller/command_duration", JtCmd, queue_size=1)
    ]

    # -------------- init pose ----------------- 
   
    # set initial calibration and native joint angles (b, s, e, w, g)
    self.load_cal()
    self.ang = [0.0] * 5               # raw servo cmds
    nom = [0, 70.3, 90.1, 30.3, 0]     # nominal angles
    for jt in range(5):
      self.ang[jt] = self.n2c(jt, nom[jt])       

    # go to pose (9 sec total)
    rospy.sleep(2.0)                   # wait for controller
    print("\x1b[1;33m")  
    print("grab_calib: Moving to canonical workspace pose .", end="", flush=True)
    self.set_servo(2, 1000)            # elbow
    print(".", end="", flush=True)
    self.set_servo(3, 500)             # wrist
    print(".", end="", flush=True)
    self.set_servo(1, 2000)            # shoulder
    print(".", end="", flush=True)
    self.set_servo(0, 2000)            # base
    print(".", end="", flush=True)
    self.set_servo(4, 500)             # gripper
    print(" done", end="")
    print("\x1b[0m") 

    # --------------- interaction ---------------

    # disable required ENTER for character input
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd) 

    # let user adjust angles then restore buffering and save values
    try:
      self.fine_calib(nom)
    finally:
      termios.tcsetattr(fd, termios.TCSADRAIN, old)
    self.save_devs()


  # convert a nominal angle to a command angle for servo using calibration
  
  def n2c(self, jt, nom):
    return self.sc[jt] * nom + self.off[jt]
   

  # set joint (b, s, e, w, g, p, t) to stored native angle

  def set_servo(self, jt, ms, inc=0.0):
    self.pub[jt].publish(JtCmd(data=math.radians(self.ang[jt] + inc), duration=ms))
    rospy.sleep(0.001 * ms)


# -------------------------------------------------------------------------

  # load YAML calibration file with servo scaling and offsets 

  def load_cal(self):  
    cfile = "/home/jetauto/Wansui/config/" + socket.gethostname() + "_servo.yaml"
    data = {}
    if os.path.exists(cfile):
      with open(cfile, 'r') as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
    self.sc = [ 
      data.get('bsc', -1.0),           # base reversed
      data.get('ssc',  1.0),
      data.get('esc',  1.0),
      data.get('wsc', -1.0),           # wrist reversed
      data.get('gsc',  1.3),           # gripper 240 vs 180 deg
      data.get('psc',  1.0),
      data.get('tsc',  1.0),
    ]
    self.off = [ 
      data.get('boff',   0.0),
      data.get('soff',   0.0),
      data.get('eoff',   0.0),
      data.get('woff',   0.0),
      data.get('goff', -40.0),         # gripper 240 vs 180 deg
      data.get('poff',   0.0),
      data.get('toff',   0.0)
    ]


  # save servo offsets to "<host>_servo.yaml" file

  def save_devs(self):
    cfile = "/home/jetauto/Wansui/config/" + socket.gethostname() + "_servo.yaml"
    data = {
      'bsc': self.sc[0],     # scale factors
      'ssc': self.sc[1],  
      'esc': self.sc[2],  
      'wsc': self.sc[3], 
      'gsc': self.sc[4], 
      'psc': self.sc[5], 
      'tsc': self.sc[6], 
      'boff':self.off[0],    # zero offsets
      'soff':self.off[1], 
      'eoff':self.off[2], 
      'woff':self.off[3],
      'goff':self.off[4],
      'poff':self.off[5],
      'toff':self.off[6]
    }
    with open(cfile, 'w') as f:
      yaml.dump(data, f, sort_keys=False)
    print("\x1b[1;32m", end="")   
    print("Saved calibration in: %s" % (cfile))
    print("-----------------------------------------------------------------------")
    print("\x1b[0m")  


# -------------------------------------------------------------------------

  # fine calibration near floor using shoulder (y) and elbow (z)

  def fine_calib(self, nom):

    # short instructions
    print("\x1b[1;32m")    
    print("-----------------------------------------------------------------------")
    print("Change fingertip position using arrow keys to calibrate")
    print("\x1b[0m")  
    print("Hit ENTER to save, q to quit ...\n")

    # accept user updates of shoulder and elbow angles
    prompt = "Fine - gripper 152 mm (6\") in front and 25 mm (1\") up: "
    print("  %s 0.0  0.0  0.0 " % (prompt), end="", flush=True)
    s0 = self.ang[1]
    e0 = self.ang[2]
    w0 = self.ang[3]
    rate = rospy.Rate(20)
    while True:

      # see if any key pressed
      while not rospy.is_shutdown():
        ch = sys.stdin.read(1)
        if ch:
          break    
        rate.sleep()

      # check for some exit condition
      if ch == 'q' or ch == 'Q':
        print("\x1b[1;31m")
        print("\nQuit!")
        print("\x1b[0m") 
        rospy.signal_shutdown()        # no file written
      if ch == '\x0A':
        break

      # alter raw cmd value (arrows prefixed by ESC)
      # counter-rotates wrist to maintain hand tilt angle
      if ch == 'A':
        self.ang[1] -= 0.2             # up = shoulder smaller
        self.ang[3] -= 0.2
      elif ch == 'B':
        self.ang[1] += 0.2             # dn = shoulder larger
        self.ang[3] += 0.2
      elif ch == 'C':  
        self.ang[2] -= 0.2             # out = elbow smaller
        self.ang[3] -= 0.2
      elif ch == 'D':  
        self.ang[2] += 0.2             # in = elbow larger
        self.ang[3] += 0.2
      else:
        continue
      print("          \r  %s%4.1f %4.1f %4.1f " % 
            (prompt, self.ang[1] - s0, self.ang[2] - e0, self.ang[3] - w0), end="", flush=True) 

      # actually adjust servos (twitch)
      self.set_servo(1, 50, -5.0)
      self.set_servo(2, 50, -5.0)
      self.set_servo(3, 50, -5.0)
      self.set_servo(1, 50)
      self.set_servo(2, 50)
      self.set_servo(3, 50)

    # alter servo offsets such that nominal angles achieve goal position
    self.off[1] = round(self.off[1] + (self.ang[1] - self.n2c(1, nom[1])), 1)
    self.off[2] = round(self.off[2] + (self.ang[2] - self.n2c(2, nom[2])), 1)
    self.off[3] = round(self.off[3] + (self.ang[3] - self.n2c(3, nom[3])), 1)
    print("\n    soff %3.1f, eoff %3.1f, woff %3.1f\n" % (self.off[1], self.off[2], self.off[3])) 


# =============================================================================

# allow node to start broadcasting

if __name__ == "__main__":
  try:
    GrabCalib()
  except rospy.ROSInterruptException:
    pass

