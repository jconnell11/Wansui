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
# Copyright 2024-2026 Etaoin Systems
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

import time, os, os.path
import Jetson.GPIO as GPIO

from jetauto_sdk import buzzer, hiwonder_servo_controller


# helper function for playing standard sounds

def PlaySFX(name):
  cmd = "aplay -q /home/jetauto/Wansui/sfx/" + name + ".wav &"
  os.system(cmd)


# ----------------------------------------------------------------------- 

# read battery voltage from digital servo
servos = hiwonder_servo_controller.HiwonderServoController('/dev/ttyTHS1', 115200)


# clear voltage estimate and beeping state
voltage = 12.0
volt_cnt = 40
volt_nag = 0
volt_hys = 0


# check average battery voltage and beep if low
# no easy way to share this value with main application
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
  v = servos.get_servo_vin(5) 
  if v is None:
    return
  v /= 1000.0
  if v > 7.5 and v < 13.0:
    voltage += 0.2 * (v - voltage)

  # if too low then start beep 
  if voltage > 10.2:                    
    volt_hys = 0
  elif volt_hys > 0 or voltage < 9.9:  # roughly 10% left
    volt_hys = 1        
    PlaySFX("beep2")  
    buzzer.on()
    volt_nag = 4                       # medium beep (200ms)


# ----------------------------------------------------------------------- 

# configure button inputs (default to BCM mode numbers)
mid_pin = 25                           # key1
side_pin = 4                           # key2
GPIO.setwarnings(False)
if not GPIO.getmode() == GPIO.BCM:
  GPIO.setmode(GPIO.BCM)
GPIO.setup(mid_pin, GPIO.IN)
GPIO.setup(side_pin, GPIO.IN)


# clear key timing state
mid_cnt = 0
side_cnt = 0


# check state of buttons on expansion board
#    mid: brief = start demo, 3 sec = shutdown
#   side: brief = stop demo,  3 sec = switch wifi

def check_keys():
  global mid_pin, side_pin, mid_cnt, side_cnt

  # check if mid button currently being pressed
  if GPIO.input(mid_pin) == GPIO.LOW:
    mid_cnt += 1
    side_cnt = 0
    if mid_cnt == 60:
      print('switch wifi')
      PlaySFX("beep4")
      buzzer.on()                      # long beep 
      time.sleep(0.4)                  
      buzzer.off()
      stop_demo()                      # ==> SHUTDOWN
      os.system('sudo shutdown -h now')
      mid_cnt = -90                    # repeat at 7.5 sec 
    return    
  
  # mid button not currently pressed
  if mid_cnt > 0:
    print('start new demo')
    PlaySFX("beep_beep")
    buzzer.on()                        # short beep
    time.sleep(0.1)                     
    buzzer.off()
    time.sleep(0.1)
    buzzer.on()                        # short beep
    time.sleep(0.1)                    
    buzzer.off()
#    stop_demo()
    start_demo()                       # ==> START DEMO
  mid_cnt = 0 

  # check if side button currently being pressed
  if GPIO.input(side_pin) == GPIO.LOW:
    side_cnt += 1
    if side_cnt == 60:
      print('switch wifi')
      PlaySFX("beep4_beep")
      buzzer.on()                      # long beep
      time.sleep(0.4)                  
      buzzer.off()
      time.sleep(0.2)
      buzzer.on()                      # short beep
      time.sleep(0.1)                    
      buzzer.off()
      stop_demo()                      # ==> SWITCH WIFI
      switch_wifi()                    
      side_cnt = -90                   # repeat at 7.5 sec
    return    
  
  # side button not currently pressed
  if side_cnt > 0:
    print('stop any demo')
    PlaySFX("beep")
    buzzer.on()                        # short beep
    time.sleep(0.1)                    
    buzzer.off()
    stop_demo()                        # ==> STOP DEMO
  side_cnt = 0 


# launch current demo 
def start_demo(): 
  os.system("screen -dm bash -c 'roslaunch wansui_vis wansui_vis.launch 2>/dev/null' &") 


# make sure demo program has stopped cleanly
def stop_demo():
  os.system("rosnode kill hmore_face wansui_vis")


# change from main to alt wifi (or vice versa)
def switch_wifi():
  if os.path.isfile("/home/jetauto/hiwonder-toolbox/main_wifi.py"):
    print("  select main wifi")
    os.system("cd /home/jetauto/hiwonder-toolbox; mv hiwonder_wifi_conf.py alt_wifi.py")
    os.system("cd /home/jetauto/hiwonder-toolbox; mv main_wifi.py hiwonder_wifi_conf.py")
  elif os.path.isfile("/home/jetauto/hiwonder-toolbox/alt_wifi.py"):
    print("  select alt wifi")
    os.system("cd /home/jetauto/hiwonder-toolbox; mv hiwonder_wifi_conf.py main_wifi.py")
    os.system("cd /home/jetauto/hiwonder-toolbox; mv alt_wifi.py hiwonder_wifi_conf.py")
  else:
    print('no wifi change')
  os.system("sudo systemctl restart hw_wifi.service")


# =========================================================================

# check buttons and voltage at 20 Hz
if __name__ == "__main__":
  os.system("pulseaudio -k")
  while True:
    check_battery()
    check_keys()
    time.sleep(0.05)

