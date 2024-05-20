// jhcWansuiAct.h : ALIA reasoner master ROS node for JetAuto Pro robot
//
// Written by Jonathan H. Connell, jconnell@alum.mit.edu
//
///////////////////////////////////////////////////////////////////////////
//
// Copyright 2024 Etaoin Systems
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
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <jhcAzureReco.h>
//#include <jhcPicoReco.h>
#include <jhcPlanarArm.h>


//= ALIA reasoner master ROS node for JetAuto Pro robot.
// assumes directories log, dump, and KB exist and are writable

class jhcWansuiAct
{
// PRIVATE MEMBER VARIABLES
private:
  // basic configuration
  ros::NodeHandle nh;

  // outgoing messages (incl. servos)
  ros::Publisher speak_pub, stare_pub, mood_pub, gaze_pub, base_pub;
  ros::Publisher neck_pub, arm_bpub, arm_spub, arm_epub, arm_wpub, arm_gpub;

  // incoming messages
  ros::Subscriber talk_sub, imu_sub, odom_sub, batt_sub;

  // ------------------------------------------------------

  // speech recognition engine 
  jhcAzureReco reco;
//  jhcPicoReco reco;

  // arm control functions
  jhcPlanarArm arm;

  // IMU zero calibration
  double tsum, rsum, t0, r0;
  int imu_cnt;

  // main loop timing
  double cyc;

  // ------------------------------------------------------

  // speech state variables 
  int talking, attention;

  // cached body orientation and status
  double pitch, roll, pct;

  // notional neck pan and next tilt command
  double pan, ntc, nt2;

  // base position and heading
  double xmap, ymap, yaw;

  // ------------------------------------------------------

  // speed factor and previous mood bits
  double sf;
  int mood0;

  // previous neck values (in degs)
  double yaw0, perr0, nt0;

  // previous arm joint commands (radians)
  double br0, sr0, er0, wr0, gr0;

  // previous and current base commands
  double mv0, rv0, sk0, mv, rv;


// PUBLIC MEMBER VARIABLES
public:
  // neck camera geometry
  double ny, nz, cfwd, cup, ct0, cp0, cr0;

  // neck motion control
  double nsp, lead;

  // base motion ramping
  double msp, mup, mdn, rsp, rup, rdn;


// PUBLIC MEMBER FUNCTIONS
public:
  // creation and initialization
  ~jhcWansuiAct ();
  jhcWansuiAct ();
  void Reset ();

  // main loop
  void Run ();


// PRIVATE MEMBER FUNCTIONS
private:
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
  double get_emotion (double& mag, double& ang, int m) const;
  void body_update ();
  void cb_imu (const sensor_msgs::Imu::ConstPtr& imu);
  void cb_batt (const std_msgs::Float32::ConstPtr& batt);

  // neck
  void neck_issue ();
  double gaze_slew (double& n2, double& nc, double nt, double rate, double lim) const;
  void neck_update ();

  // arm
  void arm_issue ();
  void arm_servos (double b, double s, double e, double w, double g);
  void arm_update ();
  void servo_angs ();

  // base
  void base_issue ();
  void alter_vel (double& v, double inc, double rt, double vn, double tup, double tdn);
  void base_update ();
  void base_stop ();
  void cb_odom (const nav_msgs::Odometry::ConstPtr& odom);

};

