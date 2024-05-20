#!/usr/bin/env python3
# encoding: utf-8

# =========================================================================
#
# ja_buttons.py : monitors JetAuto Pro expansion buttons and checks battery
#
# Written by Jonathan H. Connell, jconnell@alum.mit.edu
#
# =========================================================================
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
# =========================================================================

import time, os
import Jetson.GPIO as GPIO

from jetauto_sdk import buzzer, hiwonder_servo_controller


# ----------------------------------------------------------------------- 

# read battery voltage from digital servo
servos = hiwonder_servo_controller.HiwonderServoController('/dev/ttyTHS1', 115200)


# clear voltage estimate and beeping state
voltage = 12.0
volt_cnt = 40
volt_nag = 0
volt_hys = 0


# check average battery voltage and beep if low
# no easy way to share this with main application
def check_battery():
  global voltage, volt_cnt, volt_nag, volt_hys

  # if blip started turn off buzzer after several cycles
  if volt_nag > 0:
    volt_nag -= 1
    if volt_nag == 0:
      buzzer.off()

  # read voltage once every 2 seconds (40 cycles at 20 Hz)
  volt_cnt += 1
  if volt_cnt < 40:
    return
  volt_cnt = 0 
    
  # if not crazy add sample to IIR filter 
  v = servos.get_servo_vin(5) / 1000.0
  if v > 7.5 and v < 13.0:
    voltage += 0.2 * (v - voltage)

  # if too low then start beep 
  if voltage > 10.8:                     # nearly latched
    volt_hys = 0
  elif volt_hys > 0 or voltage < 10.2:   # roughly 10% left
    volt_hys = 1          
    buzzer.on()
    volt_nag = 4                         # medium beep (200ms)


# ----------------------------------------------------------------------- 

# configure button inputs (default to BCM mode numbers)
key1_pin = 25
key2_pin = 4
GPIO.setwarnings(False)
if not GPIO.getmode() == GPIO.BCM:
  GPIO.setmode(GPIO.BCM)
GPIO.setup(key1_pin, GPIO.IN)
GPIO.setup(key2_pin, GPIO.IN)


# clear key timing state
key1_cnt = 0
key2_cnt = 0


# check state of buttons on expansion board
# front brief = start demo, front 3 sec = <nothing>
#  back brief = stop demo,   back 3 sec = shutdown
def check_keys():
  global key1_pin, key2_pin, key1_cnt, key2_cnt

  # check if key1 currently being pressed
  if GPIO.input(key1_pin) == GPIO.LOW:
    key1_cnt += 1
    key2_cnt = 0
    if key1_cnt == 60:
      print('switch wifi mode')
      buzzer.on()
      time.sleep(0.4)                  # long beep
      buzzer.off()
    return    
  
  # key1 not currently pressed
  if key1_cnt > 0:
    print('start new demo')
    buzzer.on()
    time.sleep(0.1)                    # short beep
    buzzer.off()
    time.sleep(0.1)
    buzzer.on()
    time.sleep(0.1)                    # short beep
    buzzer.off()
#    stop_demo()
    start_demo()
  key1_cnt = 0 

  # check if key2 currently being pressed
  if GPIO.input(key2_pin) == GPIO.LOW:
    key2_cnt += 1
    if key2_cnt == 60:
      print('system shutdown')
      buzzer.on()
      time.sleep(0.4)                  # long beep
      buzzer.off()
      stop_demo()
      os.system('sudo shutdown -h now')
    return    
  
  # key2 not currently pressed
  if key2_cnt > 0:
    print('stop any demo')
    buzzer.on()
    time.sleep(0.1)                    # short beep
    buzzer.off()
    stop_demo()
  key2_cnt = 0 


# launch current demo 
def start_demo(): 
  os.system("screen -dm bash -c 'roslaunch wansui_act wansui_act.launch 2>/dev/null' &") 


# make sure demo program has stopped cleanly
def stop_demo():
  os.system("rosnode kill hmore_face wansui_act")


# =========================================================================

# check buttons and voltage at 20 Hz
if __name__ == "__main__":
  while True:
    check_battery()
    check_keys()
    time.sleep(0.05)

