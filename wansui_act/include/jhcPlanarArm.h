// jhcPlanarArm.h : forward and inverse kinematics for planar 4 DOF arm
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

//= Forward and inverse kinematics for planar 4 DOF arm.
// geometry is particularly simple and has a closed-form solution
// largely copied from mpi_arm.py for MasterPi robot
//   b = base swivel  +45 = aimed left        -45 = aimed right
//   s = shoulder     +45 = tilted forward    -45 = tilted back
//   e = elbow        +45 = down wrt se link  -45 = up wrt se link
//   w = wrist        +45 = up wrt ew link    -45 = down wrt ew link
//   g = gripper      +45 = some open         -45 = closed
// NOTE: JetAuto Pro has NEGATED versions of b, w (s, e, g are okay)
// runs open-loop with no actual joint angles (assumes perfect servo)
// uses speed to linearly adjust commands toward targets
//   current (last cmd) angles:  bc, sc, ec, wc, gc
//   next trajectory angles:     b2, s2, e2, w2, g2
//   target joint angles:        bt, st, et, wt, gt
//   current (last cmd) coords:  xc, yc, zc, tc (angle)
//   target hand coords:         xt, yt, zt, tt (angle)
// typical sequence is Ramp (next->current) followed by state sensing
// this could be encapsulated in its own ROS node ...

class jhcPlanarArm
{
// PRIVATE MEMBER VARIABLES
private:
  // finger conversion constants
  double gz, gf, gc0;

  // current and queued hand servo angles
  double gc, g2, oz;

  // current and queued arm servo angles
  double bc, sc, ec, wc;
  double b2, s2, e2, w2;

  // current Cartesian endpoint position and tilt
  double xc, yc, zc, tc;

  // hand command
  double gt, fs;

  // arm command
  double bt, st, et, wt, sp;
  double xt, yt, zt, tt;
  int firm, mode;


// PUBLIC CONFIGURATION VARIABLES
public:
  // hand geometry
  double fout, fup, jaw, sep, w0, w30, wmax;

  // arm geometry
  double sy, sz, se, ew, ez;

  // wrist camera geometry
  double cf, cr, cp0, ct0, cr0;

  // control speeds and angle limits
  double ips, dps, gps, cyc, lead;


// PUBLIC MEMBER FUNCTIONS
public:
  // creation and initialization
  ~jhcPlanarArm ();
  jhcPlanarArm ();
  void InitPose (double b, double s, double e, double w, double g);
  void Reset ();

  // hand functions
  double MaxGrip () const;
  double Width () const;                   
  double Squeeze ();                
  double Grip (double wf, double sp =1.0); 

  // arm pose functions
  void Home (double& b, double& s, double& e, double& w) const;
  double Angles (double& b, double& s, double& e, double& w) const;
  void Pose (double b, double s, double e, double w, double speed =1.0);
  double ErrAng (double b, double s, double e, double w) const;

  // arm Cartesian functions
  void Position (double& hx, double& hy, double& hz) const;
  void Orientation (double& hp, double& ht) const;
  void Move (double hx, double hy, double hz, double ht, int exact =0, double speed =1.0);
  double ErrPos (double hx, double hy, double hz) const;
  double ErrTip (double ht) const;

  // camera functions
  void Camera (double& cx, double& cy, double& cz) const;
  void View (double& cp, double& ct, double& cr) const;
  void AimFor (double& cp, double& ct, double px, double py, double pz) const;
  void Gaze (double cp, double ct, double speed =1.0);
  void LookAt (double px, double py, double pz, double speed =1.0);
  double ErrGaze (double cp, double ct) const;
  double ErrLook (double px, double py, double pz) const;

  // control functions
  int Ramp (double& b, double& s, double& e, double& w, double& g);


// PRIVATE MEMBER FUNCTIONS
private:
  // control functions
  void linear_ang ();
  void linear_xyz ();
  double v_ramp (double err, double inc) const;
  int joint_cmd (double& b, double& s, double& e, double& w);

  // kinematics
  void fwd_kin (double& hx, double& hy, double& hz, double& ht, 
                double b, double s, double e, double w, double g) const;
  void wrist_rel (double& px, double& py, double& pz, double& pt,
                  double b, double s, double e, double w, 
                  double fwd, double rise) const;
  void inv_kin (double& b, double& s, double& e, double& w,
                double hx, double hy, double hz, double ht, 
                int exact, double g) const;
  double dist_ang (double& dist, double& ang, double dx, double dy) const;
  double oblique_eqn (double a, double b, double c) const;
  double sine_law (double a, double b_int, double b) const; 

};
