// jhcPlanarArm.cpp : forward and inverse kinematics for planar 4 DOF arm
//
// Written by Jonathan H. Connell, jconnell@alum.mit.edu
//
///////////////////////////////////////////////////////////////////////////
//
// Copyright 2024-2026 Etaoin Systems
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

#include <math.h>
#include <stdio.h>

#include <jhcPlanarArm.h>


// useful conversion factors and macros

#define D2R  (M_PI / 180.0)
#define R2D  (180.0 / M_PI)


///////////////////////////////////////////////////////////////////////////
//                      Creation and Initialization                      //
///////////////////////////////////////////////////////////////////////////

//= Default destructor does necessary cleanup.

jhcPlanarArm::~jhcPlanarArm ()
{
}


//= Default constructor initializes certain values.
// should call Reset() next, before any other functions
// canonical arm angles:
//   b = base rotation CCW from front        (+45 = angled left)
//   s = shoulder degs forward from vertical (+45 = forward up)
//   e = elbow degs down from shoulder link  (+45 = angled down)
//   w = wrist degs up from elbow link       (-45 = angled down)
//   g = gripper left link degs wrt midline  (+90 = full open)
// assumes servo scaling and offset corrections applied elsewhere

jhcPlanarArm::jhcPlanarArm ()
{
  // hand geometry
  jaw  = 1.18;              // finger actuator link length (30mm) 
  sep  = 0.00;              // pivot separation - pads (29mm - 29mm)
  fout = 4.53;              // wrist to closed tips - jaw (145mm - 30mm)
  fup  = 0.16;              // tips up wrt centerline (4mm)
  wmax = 2.36;              // max opening (2 * jaw + sep = 60mm)

  // arm geometry
  sy = 0.98;                // base axis wrt turn center (25mm)
  sz = 7.64;                // shoulder axis wrt floor (194mm)
  se = 5.12;                // shoulder to elbow (130mm)
  ew = 5.12;                // elbow to wrist (130mm)
  ez = 0.0;                 // elbow zero angle wrt lower link

  // camera geometry
  cf  = 1.97;               // camera ahead of wrist (50mm)
  cr  = 1.77;               // camera above hand (45mm)

  // motion speeds
  gps  = 120.0;             // 300 max (gripper)
  dps  = 90.0;              // 300 max (arm)
  ips  = 18.0;              // 40 max (approx)
  lead = 1.0;               // servo response time vs cyc
}


//= Clear all state variables for start of next run.
// call this if finger width calibration values change

void jhcPlanarArm::Reset ()
{
  double b, s, e, w, g;

  // assume arm is in home position
  g = Home(b, s, e, w);
  InitPose(b, s, e, w, g);
  oz = 0.0;

  // clear all command parameters
  fs = 0.0;
  sp = 0.0;
  firm = 0;
  mode = 0;
}


//= Set initial joint angles for arm.
// should be called after Reset()

void jhcPlanarArm::InitPose (double b, double s, double e, double w, double g)
{
  // set current joint angles 
  bc = b;
  sc = s;
  ec = e;
  wc = w;
  gc = g;

  // set current Cartesian location
  fwd_kin(xc, yc, zc, tc, bc, sc, ec, wc, gc);  

  // set next trajectory point
  b2 = bc;
  s2 = sc;
  e2 = ec;
  w2 = wc;
  g2 = gc;

  // set joint commands
  bt = bc;
  st = sc;
  et = ec;
  wt = wc;
  gt = gc;

  // set Cartesian command
  xt = xc;
  yt = yc;
  zt = zc;
  tt = tc;
}


///////////////////////////////////////////////////////////////////////////
//                             Hand Functions                            //
///////////////////////////////////////////////////////////////////////////

//= Tell maximum gripper width in inches.

double jhcPlanarArm::MaxGrip () const
{
  return wmax;   
}


//= Tell current gripper width in inches.
// based on smooth (perfect) trajectory angle

double jhcPlanarArm::Width () const
{
  double w = 2.0 * jaw * sin(D2R * fmax(0.0, gc)) + sep;

  return fmax(0.0, w);
}


//= Tell current gripper exerted force in ounces.
// really just hysteretic: positive = holding, 0 = empty

double jhcPlanarArm::Squeeze () 
{
  if ((oz <= 0.0) && (gc <= -5.0))
    oz = 12.0;                           // 5% of 21 Kg-cm @ 30mm
  else if ((oz > 0.0) && (gc >= 60.0))   // 2" out of 2.36"
    oz = 0.0;
  return oz;
}


//= Set goal to shift to gripper width or force at some speed. 
// wf >= 0 means a specific width in inches, wf < 0 means apply force 
// returns target servo angle for convenience

double jhcPlanarArm::Grip (double wf, double speed)
{
  if (wf < 0.0)
    gt = -10.0;              // closed + 10 for force
  else if (speed > 0.0)
  {
    if (wf >= wmax)
      gt = 90.0;
    else if (wf <= sep)
      gt = 0.0;
    else
      gt = R2D * asin(0.5 * (wf - sep) / jaw);
  }
  fs = speed;
  return gt;
}


///////////////////////////////////////////////////////////////////////////
//                           Arm Pose Functions                          //
///////////////////////////////////////////////////////////////////////////

//= Canonical home pose for arm allowing travel and camera aiming
// returns default angle for gripper (parly open)

double jhcPlanarArm::Home (double& b, double& s, double& e, double& w) const
{
  b =   65.0;                // negated wrt JetAuto init_pose.py
  s =  -80.0;
  e =  115.0;                
  w = -100.0;                // negated wrt JetAuto init_pose.py
  return 30.0;               // equivalent to 0 in init_pose.py
}


//= Tell current servo joint angles wrt nomimal in degrees.
// binds base, shoulder, elbow, wrist current values
// returns canonical gripper servo angle (mostly for debugging)

double jhcPlanarArm::Angles (double& b, double& s, double& e, double& w) const
{
  b = bc;
  s = sc;
  e = ec;
  w = wc;
  return gc;
}


//= Set goal to shift to joint configuration at some speed.
// base, shoulder, elbow, wrist are in degs wrt nominal
// sp is speed as a fraction of "normal" (dps = 120)

void jhcPlanarArm::Pose (double b, double s, double e, double w, double speed)
{
  bt = b;
  st = s;
  et = e;
  wt = w;
  sp = speed;
  mode = 0;                  // angular
}


//= Tell max offset in any joint wrt reference pose in degrees.

double jhcPlanarArm::ErrAng (double b, double s, double e, double w) const
{
  double err = fabs(bc - b), ds = fabs(sc - s), de = fabs(ec - e), dw = fabs(wc - w); 

  if (ds > err)
    err = ds;
  if (de > err)
    err = de;
  if (dw > err)
    err = dw;
  return err;
}


///////////////////////////////////////////////////////////////////////////
//                        Arm Cartesian Functions                        //
///////////////////////////////////////////////////////////////////////////

//= Tell location of grip center wrt body coords in inches.
// origin is center of 4 wheels and floor (x to right, y forward)
// assumes Update() has been called first 

void jhcPlanarArm::Position (double& hx, double& hy, double& hz) const
{
  hx = xc;
  hy = yc;
  hz = zc;
}


//= Tell hand elevation wrt horizontal and pointing azimuth in degrees
// hand pan forward = +90 degrees (0 degs = right)

void jhcPlanarArm::Orientation (double& hp, double& ht) const
{
  hp = 90.0 + bc;
  ht = tc;
}


//= Set goal to shift grip center to some position at some speed.
// x, y, z are in inches, tip is deviation from horizontal in degs
// coordinate origin is center of 4 wheels and floor (x to right)
// exact > 0 says preferred tilt can be modified if needed
// sp is top speed as a fraction of normal (dps = 120, ips = 18)
// negative sp means linear path (often slower) not angular slew

void jhcPlanarArm::Move (double hx, double hy, double hz, double ht, int exact, double speed)
{
  // save basic parameters
  xt = hx;
  yt = hy;
  zt = hz;
  tt = ht;
  firm = exact;
  sp = fabs(speed);

  // figure out trajectory mode (1 = xyz, 0 = angs)
  mode = 1;                         
  if (speed < 0.0)
    return;
  mode = 0;                           
  inv_kin(bt, st, et, wt, xt, yt, zt, tt, exact, gt);
}


//= Tell distance of finger grip point from reference point in inches.
// included for local use - not need by ALIA

double jhcPlanarArm::ErrPos (double hx, double hy, double hz) const
{
  double dx = xc - hx, dy = yc - hy, dz = zc - hz;

  return sqrt(dx * dx + dy * dy + dz * dz);
}


//= Determine absolute tip error wrt reference tilt in degs.
// included for local use - not need by ALIA

double jhcPlanarArm::ErrTip (double ht) const
{
  return fabs(tc - ht);
}


///////////////////////////////////////////////////////////////////////////
//                            Camera Functions                           //
///////////////////////////////////////////////////////////////////////////

//= Tell real-world camera location in inches.
// coordinate origin is center of 4 wheels and floor (x to right)  
// included for local use by wrist camera - not need by ALIA
// NOTE: not used by Wansui (only Ganbei)

void jhcPlanarArm::Camera (double& cx, double& cy, double& cz) const
{
  double ht;

  wrist_rel(cx, cy, cz, ht, bc, sc, ec, wc, cf, cr);
}


//= Tell camera viewing direction in degrees.
// included for local use by wrist camera - not need by ALIA
// NOTE: not used by Wansui (only Ganbei)

void jhcPlanarArm::View (double& cp, double& ct, double& cr) const
{
  cp = bc;
  ct = tc;
  cr = 0.0;
}


//= Generates pan and tilt goals to aim camera at real-world target
// x,y,z in inches, origin is center of 4 wheels and floor (x to right) 
// returns pan and tilt angles based on current camera position
// Note: as pan and tilt change so will camera position -> different aim 
// included for local use by wrist camera - not need by ALIA
// NOTE: not used by Wansui (only Ganbei)

void jhcPlanarArm::AimFor (double& cp, double& ct, double px, double py, double pz) const
{
  double cx, cy, cz, dx, dy, dr, dz;

  Camera(cx, cy, cz);
  cp = R2D * atan2(-px, py - sy);
  dx = px - cx;
  dy = py - cy;
  dr = sqrt(dx * dx + dy * dy);
  dz = pz - cz;
  ct = R2D * atan2(dz, dr);
}


//= Use wrist and base joints to aim camera in some direction.
// assumes angles are in degrees wrt global coords (not pixels)
// included for local use by wrist camera - not need by ALIA
// NOTE: not used by Wansui (only Ganbei)

void jhcPlanarArm::Gaze (double cp, double ct, double speed)
{
  double b, e, s, w, t, w0, s0;

  // pan can only be affected by base angle
  b = cp;
  t = ct;

  // try adjusting wrist first to achieve tilt
  e = fmax(-45.0, fmin(ec, 45.0));
  s = fmax(-45.0, fmin(sc, 45.0));
  w0 = t + (e - ez) - (90 - s);  
  w = fmax(-45.0, fmin(w0, 45.0));
  
  // if outside limits try adjusting shoulder then elbow
  if (w != w0)
  {
    s0 = -t + w - (e - ez) + 90.0;
    s = fmax(-45.0, fmin(s0, 45.0));
    if (s != s0)
      e = -t + w + (90.0 - s) + ez;
  }
  Pose(b, s, e, w, speed);
}


//= Aim camera at some real-world target based on its current position.
// x,y,z in inches, origin is center of 4 wheels and floor (x to right) 
// included for local use by wrist camera - not need by ALIA
// NOTE: not used by Wansui (only Ganbei)

void jhcPlanarArm::LookAt (double px, double py, double pz, double speed)
{
  double pan, tilt;

  AimFor(pan, tilt, px, py, pz);
  Gaze(pan, tilt, speed);
}


//= Tell maximum angular offset (in degs) from desired pan and tilt.
// included for local use by wrist camera - not need by ALIA
// NOTE: not used by Wansui (only Ganbei)

double jhcPlanarArm::ErrGaze (double cp, double ct) const
{
  double dp = fabs(bc - cp), dt = fabs(tc - ct);
    
  return fmax(dp, dt);
}


//= Determine angular offset of gaze from some real-world target.
// x,y,z in inches, origin is center of 4 wheels and floor (x to right) 
// included for local use by wrist camera - not need by ALIA
// NOTE: not used by Wansui (only Ganbei)

double jhcPlanarArm::ErrLook (double px, double py, double pz) const
{
  double pan, tilt;

  AimFor(pan, tilt, px, py, pz);
  return ErrGaze(pan, tilt);
}


///////////////////////////////////////////////////////////////////////////
//                           Control Functions                           //
///////////////////////////////////////////////////////////////////////////

//= Get new joint servo angle commands based on targets and speeds.
// uses bt, st, et, wt, gt targets to produce b, s, e, w, g servo cmds
// typically called at beginning of new cycle (before reading sensors)
// returns 1 if okay, 0 if some joint angle clipping

int jhcPlanarArm::Ramp (double& b, double& s, double& e, double& w, double& g, double cyc)
{
  // assume perfect servo following so all incremental targets achieved
  bc = b2;
  sc = s2;
  ec = e2;
  wc = w2;
  gc = g2;
  fwd_kin(xc, yc, zc, tc, bc, sc, ec, wc, gc);   // current Cartesian location

  // get new gripper angle (must compute before new arm joints)
  g2 = gc + v_ramp(gc - gt, fs * gps * cyc);
  g2 = fmax(-10.0, fmin(g2, 75.0));
  g = gc + lead * (g2 - gc);                     // desired canonical angle

  // compute new arm joint angles then bind servo commands
  if (mode <= 0)
    linear_ang(cyc);
  else 
    linear_xyz(cyc);   
  return joint_cmd(b, s, e, w);
}


//= Linearly ramp angles in current arm pose toward target pose.
// assumes target is bt, st, et, wt and current angles are bc, sc, ec, wc
// computes incremental target pose b2, s2, e2, w2 to achieve by next cycle

void jhcPlanarArm::linear_ang (double cyc)
{
  double db = bc - bt, ds = sc - st, de = ec - et, dw = wc - wt; 
  double top, da = sp * dps * cyc;

  // find signed joint errors and max magnitude
  top = fmax(0.1, fabs(db));
  top = fmax(top, fabs(ds)); 
  top = fmax(top, fabs(de));  
  top = fmax(top, fabs(dw));   

  // adjust joint speeds so all finish at the same time
  b2 = bc + v_ramp(db, da * fabs(db) / top);
  s2 = sc + v_ramp(ds, da * fabs(ds) / top);
  e2 = ec + v_ramp(de, da * fabs(de) / top);
  w2 = wc + v_ramp(dw, da * fabs(dw) / top);
}


//= Linearly ramp current Cartesian grip point toward target location.
// assumes target is xt, yt, zt, tt and current angles are bc, sc, ec, wc
// computes incremental target pose b2, s2, e2, w2 to achieve by next cycle
// needs g2 computed first to give consistent location

void jhcPlanarArm::linear_xyz (double cyc)
{
  double dx = xc - xt, dy = yc - yt, dz = zc - zt, dt = tc - tt;
  double lim, dp, pcyc, tcyc, cnt, pf, tf, top, xf, yf, zf;
  double inc, ang, x2, y2, z2, t2, dev, f;
  
  // compute move per cycle and max servo slew
  dp = sp * ips * cyc;   
  lim = 0.5 * 300.0 * cyc; 

  // scale translation and rotation so they finish simultaneously
  pcyc = ErrPos(xt, yt, zt) / dp;
  tcyc = fabs(dt) / lim;
  cnt = fmax(0.1, fmax(pcyc, tcyc));
  pf = pcyc / cnt;
  tf = tcyc / cnt; 

  // adjust coordinate speeds for direction of travel
  top = fmax(0.1, fabs(dx));
  top = fmax(top, fabs(dy));
  top = fmax(top, fabs(dz));
  xf = pf * fabs(dx) / top;
  yf = pf * fabs(dy) / top;
  zf = pf * fabs(dz) / top;

  // step in xyzt and find corresponding shift in joint angles
  inc = dp;
  ang = lim;
  while (1)
  {
    // shift by balanced speed-limited amount
    x2 = xc + v_ramp(dx, inc * xf);
    y2 = yc + v_ramp(dy, inc * yf);
    z2 = zc + v_ramp(dz, inc * zf);
    t2 = tc + v_ramp(dt, ang * tf);

    // find joint angles then check if angular speed okay
    inv_kin(b2, s2, e2, w2, x2, y2, z2, t2, firm, g2);
    dev = ErrAng(b2, s2, e2, w2);
    if (dev <= lim)
      break;

    // if rotation too fast then try a smaller step   
    if ((inc > 0.05) || (ang > 0.5))
    {   
      inc *= 0.7;                  
      ang *= 0.7; 
      continue;
    }

    // give up and do joint interpolation instead
    f = lim / dev;
    b2 = bc + f * (b2 - bc);
    s2 = sc + f * (s2 - sc);
    e2 = ec + f * (e2 - ec);
    w2 = wc + f * (w2 - wc);
    break;
  }
}


//= Get amount to shift command to fix "err" but limit to "inc".

double jhcPlanarArm::v_ramp (double err, double inc) const
{
  if (err < 0.0)
    return fmin(inc, -err);
  return -fmin(inc, err);
}


//= Limit joints to valid range and produce servo commands with overshoot.
// b2, s2, e2, w2 values are proposed new trajectory point 
// returns 1 if okay, 0 if some joint angle clipping

int jhcPlanarArm::joint_cmd (double& b, double& s, double& e, double& w) 
{
  double bb = b2, ss = s2, ee = e2, ww = w2, lim = 115.0;   
  int ok = 1;

  // limit trajectory point to feasible servo range
  b2 = fmax( -lim, fmin(b2, lim));
  s2 = fmax(-95.0, fmin(s2, lim));     // was -lim
  e2 = fmax( -lim, fmin(e2, lim));
  w2 = fmax( -lim, fmin(w2, lim));

  // set command some distance beyond new point (but same speed)
  b = bc + lead * (b2 - bc);
  s = sc + lead * (s2 - sc);
  e = ec + lead * (e2 - ec);
  w = wc + lead * (w2 - wc);

  // see if any angles were changed
  if ((b2 != bb) || (s2 != ss) || (e2 != ee) || (w2 != ww))
    return 0;
  return 1;
}


///////////////////////////////////////////////////////////////////////////
//                               Kinematics                              //
///////////////////////////////////////////////////////////////////////////

// Forward kinematics. 
// convert joint angles in degs to world position of grasp center
// returns x, y, z, in inches and gripper tilt in degs

void jhcPlanarArm::fwd_kin (double& hx, double& hy, double& hz, double& ht, 
                            double b, double s, double e, double w, double g) const
{
  double wf = fout + jaw * cos(D2R * fmax(0.0, g));

  wrist_rel(hx, hy, hz, ht, b, s, e, w, wf, fup);
}


//= Find location of a point with some displacement wrt gripper axis.
// "fwd" is along axis of fingers, "up" is orthogonal to that 
// used both for fingertips (rise = 0) and for camera (rise > 0)

void jhcPlanarArm::wrist_rel (double& px, double& py, double& pz, double& pt, 
                              double b, double s, double e, double w, 
                              double fwd, double rise) const
{
  double sup, eup, srads, erads, wr, wz, tup, trads, gr, gz, brads, gx, gy;

  // determine radial coords of wrist wrt base joint
  sup = 90 - s;
  eup = sup - (e - ez);  
  srads = D2R * sup;
  erads = D2R * eup;          
  wr = se * cos(srads) + ew * cos(erads);
  wz = se * sin(srads) + ew * sin(erads);

  // adjust for current length of gripper
  tup = eup + w;   
  trads = D2R * tup;
  gr = wr + fwd * cos(trads) - rise * sin(trads); 
  gz = wz + fwd * sin(trads) + rise * cos(trads);

  // convert endpoint location to Cartesian coords
  brads = D2R * b;
  gx = -gr * sin(brads);
  gy =  gr * cos(brads);

  // add in offset for base of arm
  px = gx;
  py = gy + sy;
  pz = gz + sz;
  pt = tup;
}


//= Inverse kinematics.
// determine joint angles for given world position of grasp center
// also takes desired hand tilt "t" in degs and whether it is fixed
// needs gripper servo angle "g" in degs to find grasp center
// configures arm for closest approach to goal given constraints
// returns base, shoulder, elbow, and wrist angles in degs

void jhcPlanarArm::inv_kin (double& b, double& s, double& e, double& w,
                            double hx, double hy, double hz, double ht, 
                            int exact, double g) const
{
  double gr, gz, sg, gup, wf = fout + jaw * cos(D2R * fmax(0.0, g));
  double sw_max, trads, wr, wz, sw, wup, f_int, s_int, w_int, e_int, eup, sup;

  // cylindrical coords of goal (gr gz) wrt shoulder
  dist_ang(gr, b, hy - sy, -hx);
  gz = hz - sz;
  dist_ang(sg, gup, gr, gz);

  // if super far consider making whole arm straight (e = 0, w = 0)
  sw_max = se + ew;
  if (exact <= 0)
    if (sg >= (sw_max + wf))  
    {
      s = 90 - gup; 
      e = ez; 
      w = 0.0;     
      return;                // super far adjust
    }

  // needed coords of wrist (wr wz) wrt shoulder for fixed tilt 
  trads = D2R * ht;
  wr = gr - wf * cos(trads) + fup * sin(trads); 
  wz = gz - wf * sin(trads) - fup * cos(trads);
  dist_ang(sw, wup, wr, wz);

  // if too far consider making lower arm straight (e = 0)
  if (sw >= sw_max)
  {
    // find closest finger approach given fixed tilt
    if (exact > 0)
    {
      f_int = gup - ht;
      s_int = sine_law(wf, f_int, sw_max);
      sup = gup + s_int;
      s = 90.0 - sup;
      e = ez;
      w = ht - sup;
      return;                // super far fixed
    }

    // adjust tilt to hit goal (known closer than super far)
    w_int = oblique_eqn(sw_max, wf, sg);
    w = w_int - 180.0;
    s_int = sine_law(wf, w_int, sg);
    sup = gup + s_int;
    if (ht > 0.0)            // wrist down      
    {
      w = -w;                
      sup = gup - s_int;        
    }
    s = 90.0 - sup;
    e = ez;
    return;                  // reachable adjust
  }

  // find elbow bend (up) for reachable point using fixed tilt
  e_int = oblique_eqn(se, ew, sw);
  e = (180.0 - e_int) + ez;
  s_int = sine_law(ew, e_int, sw);
  sup = wup + s_int;                                   
  eup = sup - (e - ez);
  s = 90.0 - sup;
  w = ht - eup;              // reachable fixed
}


//= Find hypotenuse and base angle for a point in rectangular coords.

double jhcPlanarArm::dist_ang (double& dist, double& ang, double dx, double dy) const
{
  dist = sqrt(dx * dx + dy * dy);
  ang = R2D * atan2(dy, dx);
}


//= Find interior angle C opposite side c in a-b-c triangle.
//    c^2 = a^2 + b^2 - 2 * a * b * cos(C)

double jhcPlanarArm::oblique_eqn (double a, double b, double c) const
{
  return(R2D * acos((a * a + b * b - c * c) / (2.0 * a * b)));
}


//= Find interior angle A opposite side a in a-b-c triangle.
//    sin(A) / a = sin(B) / b = sin(C) / c

double jhcPlanarArm::sine_law (double a, double b_int, double b) const
{
  return(R2D * asin(a * sin(D2R * b_int) / b));
}
