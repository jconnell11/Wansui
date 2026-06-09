#!/usr/bin/env python3
# encoding: utf-8

# =============================================================================
#
# servo_calib.py : calibration of JetAuto arm servo angles
#
# Written by Jonathan H. Connell, jconnell@alum.mit.edu
#
# -----------------------------------------------------------------------------
#
# Copyright 2025-2026 Etaoin Systems
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

class ServoCalib:

  # constructor initializes some items
  def __init__(self):

    # enroll with ROS as a node
    rospy.init_node('servo_calib', anonymous=True)
    
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
   
    # set initial calibration and native joint angles (b, s, e, w, g, p, t)
    self.load_cal()
    self.ang = [0.0] * 7
    nom = [0, 0, 0, 0, 30, -90, 0]
    for jt in range(7):
      self.ang[jt] = self.n2c(jt, nom[jt])

    # go to pose (9 sec total)
    rospy.sleep(2.0)                   # wait for controller
    print("\x1b[1;33m")  
    print("servo_calib: Commanding initial pose .", end="", flush=True)
    self.set_servo(2, 1000)            # elbow
    print(".", end="", flush=True)
    self.set_servo(3, 500)             # wrist
    print(".", end="", flush=True)
    self.set_servo(1, 2000)            # shoulder
    print(".", end="", flush=True)
    self.set_servo(0, 2000)            # base
    print(".", end="", flush=True)
    self.set_servo(4, 500)             # gripper
    print(".", end="", flush=True)
    self.set_servo(6, 500)             # neck tilt
    print(".", end="", flush=True)
    self.set_servo(5, 500)             # neck pan
    print(" done", end="")
    print("\x1b[0m") 

    # --------------- interaction ---------------

    # disable required ENTER for character input
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd) 

    # let user adjust angles then restore buffering and save values
    try:
      self.run_steps()
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
    cfile = "/home/jetauto/Wansui/config/" + socket.gethostname() + "_servo"
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
    with open(cfile + ".yaml", 'w') as f:
      yaml.dump(data, f, sort_keys=False)
    os.system("cp " + cfile + ".yaml " + cfile + "_0.yaml")    # backup copy
    print("\x1b[1;32m", end="")   
    print("Saved calibration in: %s" % (cfile + ".yaml"))
    print("-----------------------------------------------------------------------")
    print("\x1b[0m")  


# -------------------------------------------------------------------------

  # keyboard interaction to determine angular offsets

  def run_steps(self):
    # short instructions
    print("\x1b[1;32m")    
    print("-----------------------------------------------------------------------")
    print("Change joint angles using arrow keys to calibrate")
    print("\x1b[0m")  
    print("Hit ENTER to go to next step, q to quit ...\n")

    # shoulder calibration  
    n1, c1 = self.solicit(1,  0, "Shoulder - exactly vertical:   ")
    n2, c2 = self.solicit(1, 45, "           45 degrees forward: ")
    self.swing(1, n1, c1, n2, c2)

    # elbow calibration
    n1, c1 = self.solicit(2,  0, "Elbow - 45 degrees forward: ")
    n2, c2 = self.solicit(2, 45, "        exactly horizontal: ")
    self.swing(2, n1, c1, n2, c2)

    # wrist calibration 
    n1, c1 = self.solicit(3,   0, "Wrist - exactly horizontal:  ")
    n2, c2 = self.solicit(3, -45, "        45 degrees downward: ")
    self.swing(3, n1, c1, n2, c2)

    # gripper calibration
    n1, c1 = self.solicit(4, 30, "Gripper - tips 30 mm (1.2\") under pressure:  ")
    n2, c2 = self.solicit(4,  0, "          tips just touching under pressure: ")
    self.swing(4, n1, c1, n2, c2)    

    # base calibration 
    n1, c1 = self.solicit(0,   0, "Base - fingertips on centerline:    ")
    n2, c2 = self.solicit(0, -45, "       tips right by 230 mm (9.0\"): ")
    self.swing(0, n1, c1, n2, c2)

    # neck pan calibration
    n1, c1 = self.solicit(5, -90, "Pan - looking exactly right:  ")
    n2, c2 = self.solicit(5,   0, "      looking straight ahead: ")
    self.swing(5, n1, c1, n2, c2)

    # neck tilt calibration
    n1, c1 = self.solicit(6,   0, "Tilt - exactly horizontal:  ")
    n2, c2 = self.solicit(6, -45, "       45 degrees downward: ")
    self.swing(6, n1, c1, n2, c2)


  # adjust joint angle to get good fit to nominal angle using arrow keys 
  # stops whole program if 'q', exits if 'enter'

  def solicit(self, jt, nom, prompt):

    # set servo to initial native angle 
    ang0 = self.ang[jt]
    self.ang[jt] = self.n2c(jt, nom)
    if abs(ang0 - self.ang[jt]) < 10:   
      self.set_servo(jt, 50)    
    else:
      self.set_servo(jt, 500)          # slow

    # accept user updates of command angle
    print("  %s%5.1f " % (prompt, self.ang[jt]), end="", flush=True)
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

      # alter deviation value (arrows prefixed by ESC)
      if ch == 'A' or ch == 'C':
        self.ang[jt] += 0.5
      elif ch == 'B' or ch == 'D':  
        self.ang[jt] -= 0.5
      else:
        continue
      print("          \r  %s%5.1f " % (prompt, self.ang[jt]), end="", flush=True)    

      # actually adjust servo (twitch)
      self.set_servo(jt, 50, 5.0)
      self.set_servo(jt, 50)

    # end prompt line then return target and equivalent command
    print()
    return nom, self.ang[jt]


  # determine scaling and offset for servo based on v1, v2, and range
  # range is expressed as nominal angles associated with v1 and v2

  def swing(self, jt, n1, c1, n2, c2):
    self.sc[jt]  = round((c1 - c2) / (n1 - n2), 3)
    self.off[jt] = round(0.5 * (c1 + c2 - self.sc[jt] * (n1 + n2)), 1)
    print("    sc %5.3f, off %3.1f\n" % (self.sc[jt], self.off[jt])) 


# =============================================================================

# allow node to start broadcasting

if __name__ == "__main__":
  try:
    ServoCalib()
  except rospy.ROSInterruptException:
    pass

