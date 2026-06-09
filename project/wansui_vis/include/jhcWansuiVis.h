// jhcWansuiVis.h : ALIA reasoner ROS node for JetAuto Pro with vision
//
// Written by Jonathan H. Connell, jconnell@alum.mit.edu
//
///////////////////////////////////////////////////////////////////////////
//
// Copyright 2024-2025 Etaoin Systems
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// 
///////////////////////////////////////////////////////////////////////////

#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp> 

#include <jhcAzureReco.h>
//#include <jhcPicoReco.h>
#include <jhcPlanarArm.h>


//= ALIA reasoner master ROS node for JetAuto Pro robot.
// assumes directories log, dump, and KB exist and are writable

class jhcWansuiVis
{
// PRIVATE MEMBER VARIABLES
private:
  // basic configuration
  ros::NodeHandle nh;

  // outgoing messages (incl. servos)
  ros::Publisher speak_pub, stare_pub, gaze_pub, mood_pub, base_pub;
  ros::Publisher pan_pub, tilt_pub, arm_bpub, arm_spub, arm_epub, arm_wpub, arm_gpub;

  // incoming messages
  ros::Subscriber talk_sub, imu_sub, odom_sub, batt_sub, mic_sub, col_sub, rng_sub;

  // ------------------------------------------------------

  // speech recognition engine 
  jhcAzureReco reco;
//  jhcPicoReco reco;

  // arm control functions + actual gripper
  jhcPlanarArm arm;

  // IMU zero calibration
  double tsum, rsum, t0, r0;
  int imu_cnt;

  // main loop timing
  double cyc, tick;
  int rcnt;

  // ------------------------------------------------------

  // speech state variables 
  int talking, attention;

  // cached body orientation and status
  double pitch, roll, volts, pct;
  int sdir;

  // neck pan and tilt commands
  double npc, np2, ntc, nt2;

  // previous location and heading
  double xpos, ypos, yaw;

  // base cumulative motion and position
  double trav, wind, xmap, ymap;

  // ------------------------------------------------------

  // speed factor and previous mood bits
  double sf;
  int mood0;

  // previous neck values (in degs)
  double np0, nt0;

  // previous arm joint commands (in degs)
  double b0, s0, e0, w0, g0;

  // previous and current base commands
  double mv0, rv0, sk0, mv, rv;

  // ------------------------------------------------------

  // debugging output images
  sensor_msgs::Image::ConstPtr ckeep, rkeep;
  cv::Mat cam, map;
  int mtitle, show;


// PRIVATE CONFIGURATION VARIABLES
private:
  // neck servo scaling and offsets
  double psc, tsc, poff, toff;

  // arm servo scaling and offsets
  double bsc, ssc, esc, wsc, gsc;
  double boff, soff, eoff, woff, goff;

  // neck camera geometry
  double ny, nz, clf, cfwd, cup;


// PUBLIC MEMBER VARIABLES
public:
  // neck motion control
  double nsp, lead;

  // base motion ramping
  double msp, mup, mdn, rsp, rup, rdn;


// PUBLIC MEMBER FUNCTIONS
public:
  // creation and initialization
  ~jhcWansuiVis ();
  jhcWansuiVis ();
  void Reset ();

  // main loop
  void Run ();


// PRIVATE MEMBER FUNCTIONS
private:
  // creation and initialization
  void show_init ();
  void calibrate (const char *dir, const char *rname);

  // main loop
  void start ();
  void issue ();
  void update ();
  void shutdown ();

  // speech
  void tts_issue ();
  void reco_update ();
  void cb_talk (const std_msgs::Bool::ConstPtr& busy);

  // body
  void body_issue ();
  void body_update () const;
  void cb_imu (const sensor_msgs::Imu::ConstPtr& imu);
  void cb_batt (const std_msgs::Float32::ConstPtr& batt);
  void cb_mic (const std_msgs::Int32::ConstPtr& dir);  

  // neck
  void neck_issue ();
  double gaze_slew (double& n2, double& nc, double nt, double rate, double lim) const;
  void neck_update () const;

  // arm
  void arm_issue ();
  void arm_servos (double b, double s, double e, double w, double g);
  void arm_update ();

  // base
  void base_issue ();
  void alter_vel (double& v, double inc, double rt, double vn, double tup, double tdn) const;
  void base_update ();
  void base_stop ();
  void cb_odom (const nav_msgs::Odometry::ConstPtr& odom);

  // vision
  void status_imgs ();
  void cb_color (const sensor_msgs::Image::ConstPtr& img);
  void cb_range (const sensor_msgs::Image::ConstPtr& img);

};

