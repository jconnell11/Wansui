// jhcWansuiAct.cpp : ALIA reasoner master ROS node for JetAuto Pro robot
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

#include <stdio.h>
#include <math.h>
#include <signal.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/xmlrpc_manager.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <hiwonder_servo_msgs/CommandDuration.h>
              
#include <alia_act.h>                  

#include <jhcWansuiAct.h>


// useful conversion factors

#define D2R  (M_PI / 180.0)
#define R2D  (180.0 / M_PI)
#define I2M  0.0254
#define M2I  39.37


///////////////////////////////////////////////////////////////////////////

// signal-safe flag for whether "rosnode kill" or ctrl-C is received

sig_atomic_t volatile wansui_kill = 0;


// replacement SIGINT handler (ctrl-C)

void stop_handler (int sig)
{
  wansui_kill = 1;
}


//= replacement "shutdown" XMLRPC callback (from "rosnode kill")

void cb_kill (XmlRpc::XmlRpcValue& p, XmlRpc::XmlRpcValue& res)
{
  if (p.getType() == XmlRpc::XmlRpcValue::TypeArray)
    if (p.size() > 1)
      wansui_kill = 1; 
  res = ros::xmlrpc::responseInt(1, "", 0);
}


// install alternate signalling and start up ROS

void ros_guard (int argc, char *argv[], const char *node)
{
  // override SIGINT handler
  ros::init(argc, argv, node, ros::init_options::NoSigintHandler);
  signal(SIGINT, stop_handler);

  // override XMLRPC shutdown
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", cb_kill);
}


///////////////////////////////////////////////////////////////////////////
//                          Program Entry Point                          //
///////////////////////////////////////////////////////////////////////////

//= Start up ALIA reasoner master ROS node for JetAuto Pro robot.

int main (int argc, char *argv[])
{
  ros_guard(argc, argv, "wansui_act");
  jhcWansuiAct w;
  w.Reset();
  w.Run();
  return 0;
}


///////////////////////////////////////////////////////////////////////////
//                      Creation and Initialization                      //
///////////////////////////////////////////////////////////////////////////

//= Default destructor does necessary cleanup.

jhcWansuiAct::~jhcWansuiAct ()
{
  base_stop();
}


//= Default constructor initializes certain values.
// should call Reset() next, before Run()

jhcWansuiAct::jhcWansuiAct ()
{
  // outgoing messages
  speak_pub = nh.advertise<std_msgs::String>("speak", 1);
  stare_pub = nh.advertise<std_msgs::Bool>("stare", 1);
  mood_pub  = nh.advertise<geometry_msgs::Point>("mood", 1);
  gaze_pub  = nh.advertise<geometry_msgs::Point>("gaze", 1); 
  base_pub  = nh.advertise<geometry_msgs::Twist>("jetauto_controller/cmd_vel", 1);

  // outgoing servo messages
  neck_pub = nh.advertise<hiwonder_servo_msgs::CommandDuration>("n_joint_controller/command_duration", 1);
  arm_bpub = nh.advertise<hiwonder_servo_msgs::CommandDuration>("joint1_controller/command_duration", 1);
  arm_spub = nh.advertise<hiwonder_servo_msgs::CommandDuration>("joint2_controller/command_duration", 1);
  arm_epub = nh.advertise<hiwonder_servo_msgs::CommandDuration>("joint3_controller/command_duration", 1);
  arm_wpub = nh.advertise<hiwonder_servo_msgs::CommandDuration>("joint4_controller/command_duration", 1);
  arm_gpub = nh.advertise<hiwonder_servo_msgs::CommandDuration>("r_joint_controller/command_duration", 1);

  // incoming messages
  talk_sub = nh.subscribe<std_msgs::Bool>("talking", 10, &jhcWansuiAct::cb_talk, this);
  imu_sub  = nh.subscribe<sensor_msgs::Imu>("imu", 10, &jhcWansuiAct::cb_imu, this);
  odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 10, &jhcWansuiAct::cb_odom, this);
  batt_sub = nh.subscribe<std_msgs::Float32>("voltage", 10, &jhcWansuiAct::cb_batt, this);

  // neck camera geometry
  ny   = -5.28;              // neck in front of wheel center (134mm)
  nz   = 19.21;              // neck axis height above floor (488mm)
  cup  =  3.20;              // camera above axis (81mm)
  cfwd =  0.79;              // camera in front of axis (20mm)
  cp0  =  0.0;               // camera pan offset  
  ct0  =  0.0;               // camera tilt offset
  cr0  =  0.0;               // camera roll offset

  // neck control
  nsp  = 90.0;               // pan and tilt speed (dps)
  lead =  1.0;               // servo response time vs cyc

  // base motion ramping  
  msp =  18.0;               // full move speed (ips)
  mup =   0.5;               // secs to full move
  mdn =   1.5;               // secs to move stop
  rsp = 120.0;               // full turn speed (dps)
  rup =   0.5;               // secs to full turn
  rdn =   1.5;               // secs to turn stop
}


//= Initialize all variables for start of next run.

void jhcWansuiAct::Reset ()
{
  // arm parameters
  arm.Reset();

  // IMU zero calibration
  tsum = 0.0;
  rsum = 0.0;
  t0 = 0.0;
  r0 = 0.0;
  imu_cnt = 0;

  // speech activity status
  talking = -1;
  attention = -1;

  // body orientation and power status
  pitch = 0.0;
  roll  = 0.0;      
  pct   = 100.0; 

  // speed factor and previous mood bits
  sf = 1.0;
  mood0 = 0;

  // current neck pan wrt travel direction
  nt0 = 0.0;
  pan = 0.0;
  perr0 = 0.0;

  // base location and heading
  xmap = 0.0;
  ymap = 0.0;
  yaw  = 0.0;
  yaw0 = 0.0;

  // previous and current base speeds
  mv0 = 0.0;
  rv0 = 0.0;
  sk0 = 0.0;
  mv  = 0.0;
  rv  = 0.0;
}


///////////////////////////////////////////////////////////////////////////
//                               Main Loop                               //
///////////////////////////////////////////////////////////////////////////

//= Enter reasoning loop with user interaction and robot body.
// should call Reset() before this

void jhcWansuiAct::Run ()
{
  ros::Rate rate(30.0);
  cyc = 0.0333;
  start();
  while (ros::ok() && (wansui_kill <= 0))
  {  
    issue();
    update();
    if (alia_think() <= 0)
      break;
    ros::spinOnce();         // allow callbacks to run
    rate.sleep();
  }
  shutdown();                // called even for kill
}


//= Configure and start up all components.

void jhcWansuiAct::start ()
{
  char rname[80];
  int rc, wait = 0;

  // enable microphone and stop any overall motion
  rc = system("sudo systemctl restart pulseaudio.service");
  rc = system("pactl set-source-mute @DEFAULT_SOURCE@ 0"); 
  base_stop();  

  // wait for IMU to calibrate and graphical head to start
  while ((imu_cnt <= 0) || (wait < 4))         
  {
    ros::spinOnce();         // callback increments imu_cnt
    sleep(1); 
    wait++;
  }             

  // get starting arm and neck poses
  servo_angs();       

  // start reasoner with working directory and robot name
  gethostname(rname, 80);
  strcat(rname, " Wansui");
  alia_reset("/home/jetauto/Wansui", rname, "wansui_act");

  // start speech recognizer (needs all_names.txt from reasoner)
  std::string rcfg = ros::package::getPath("wansui_act");
  if (reco.Start(rcfg.c_str()) <= 0)
  {
    printf(">>> Could not start speech recognition!\n");
    ROS_ERROR("jhcWansuiAct: Could not start speech recognition"); 
  }
} 


//= Transfer commands from ALIA reasoner to actuators.

void jhcWansuiAct::issue ()
{
  tts_issue();
  body_issue();
  neck_issue();
  arm_issue();
  base_issue();
}


//= Get data from sensors and transfer to ALIA reasoner.

void jhcWansuiAct::update ()
{
  reco_update();
  body_update();
  neck_update();
  arm_update();
  base_update();
}


//= Cleanly stop all actions and save data.

void jhcWansuiAct::shutdown ()
{
  int rc;

  // stop reasoning engine and unmute microphone
  alia_done(1);
  rc = system("pactl set-source-mute @DEFAULT_SOURCE@ 0"); 

  // stop all actuator motions
  base_stop();
}


///////////////////////////////////////////////////////////////////////////
//                                Speech                                 //
///////////////////////////////////////////////////////////////////////////

//= Possibly speak output text or change eye center color. 

void jhcWansuiAct:: tts_issue ()
{
  std_msgs::String msg;
  const char *output;

  output = alia_spout();
  if (*output != '\0')
  {
    msg.data = output;
    speak_pub.publish(msg);
  }
}


//= Get any speech recognition results and set status flags.

void jhcWansuiAct::reco_update ()
{
  char text[200];

  if ((alia_hear = reco.Status()) == 2)
    alia_spin(reco.Heard(text));
  alia_talk = talking;
}


//= Mute audio input when talking and remember state.

void jhcWansuiAct::cb_talk (const std_msgs::Bool::ConstPtr& busy)
{
  char cmd[80];
  int rc;

  talking = ((busy->data) ? 1 : 0);
  sprintf(cmd, "pactl set-source-mute @DEFAULT_SOURCE@ %d &", talking);
  rc = system(cmd);   
}


///////////////////////////////////////////////////////////////////////////
//                                 Body                                  //
///////////////////////////////////////////////////////////////////////////

//= Possibly change eye color and overall facial expression.
// [ surprised angry scared happy : unhappy bored lonely tired ]
// face message = (magnitude angle transition-time)

void jhcWansuiAct::body_issue ()
{
  std_msgs::Bool eye;
  geometry_msgs::Point expr;

  // turn eyes blue when listening (no attention word needed)
  if (alia_attn != attention)
  {
    eye.data = ((alia_attn > 1) ? true : false);
    stare_pub.publish(eye);
    attention = alia_attn;
  }

  // check if emotional state has changed
  if (alia_mood != mood0)
  {
    // modulate action speeds based on emotion
    sf = 1.0;
    if ((alia_mood & 0x21) != 0)       // scared or tired
      sf = 0.8;
    if ((alia_mood & 0x0140) != 0)     // very happy or angry             
      sf *= 1.2;

    // pick facial expression based on mood bits
    expr.z = get_emotion(expr.x, expr.y, alia_mood);                                
    mood_pub.publish(expr);
    mood0 = alia_mood;
  }
}


//= Convert ALIA mood bits to emotion magnitude and angle.
// checks multiple cases in priority order
// returns transition time (secs)
// <pre>
//        120 unhappy  surprised 60
//                 \    /
//  180 scared ---- rest ---- happy 0
//                 /    \
//         240 angry   excited 300
// </pre>

double jhcWansuiAct::get_emotion (double& mag, double& ang, int m) const
{
  double f0, f1;
 
  // surprised (0x80) is very simple  
  if ((m & 0x80) != 0) 
  {
    ang = 60.0;
    mag = 1.0;
    return 0.2;                                  // very fast
  } 

  // if angry (0x40) mix with scared (0x20)
  if ((m & 0x40) != 0)         
  {    
    f0 = (((m & 0x4000) == 0) ? 0.5 : 1.0);
    f1 = (((m & 0x20) == 0) ? 0.0 : (((m & 0x2000) == 0) ? 0.5 : 1.0));   
    ang = (f0 * 240.0 + f1 * 180.0) / (f0 + f1);
    mag = fmax(f0, f1);
    return 0.5;                                  // fast
  }

  // if scared (0x20) mix with unhappy (0x08) 
  if ((m & 0x20) != 0)
  {  
    f0 = (((m & 0x2000) == 0) ? 0.5 : 1.0);
    f1 = (((m & 0x08) == 0) ? 0.0 : (((m & 0x0800) == 0) ? 0.5 : 1.0));  
    ang = (f0 * 180.0 + f1 * 120.0) / (f0 + f1);
    mag = fmax(f0, f1);
    return 1.0;
  }

  // happy (0x10) is simple
  if ((m & 0x10) != 0)
  {                    
    ang = 0.0;
    mag = (((m & 0x1000) == 0) ? 0.5 : 1.0);
    return 1.0;
  }

  // unhappy (0x08) is simple
  if ((m & 0x08) != 0)
  {                    
    ang = 120.0;
    mag = (((m & 0x0800) == 0) ? 0.5 : 1.0);
    return 1.0;
  }

  // default to neutral (angle irrelevant)
  ang = 0.0;
  mag = 0.0;
  return 2.0;                                    // slow
}


//= Get battery capacity and body attitude.

void jhcWansuiAct::body_update ()
{
  alia_batt = pct;
  alia_tilt = pitch;
  alia_roll = roll;
}


//= Analyse IMU readings to give heading and body attitude.

void jhcWansuiAct::cb_imu (const sensor_msgs::Imu::ConstPtr& imu)
{
  double qx, qy, qz, qw, qx2, qy2, qz2, qw2; 

  // get elements of quaternion
  qx = imu->orientation.x;
  qy = imu->orientation.y;
  qz = imu->orientation.z;
  qw = imu->orientation.w;
  qx2 = qx * qx;
  qy2 = qy * qy;
  qz2 = qz * qz;
  qw2 = qw * qw;

  // resolve into yaw, pitch, and roll (from Wikipedia)
  yaw   =  R2D * atan2(2.0 * (qx * qy + qw * qz), qw2 + qx2 - qy2 - qz2);
  roll  =  R2D * atan2(2.0 * (qy * qz + qw * qx), qw2 - qx2 - qy2 + qz2) - r0;
  pitch = -R2D * asin( 2.0 * (qx * qz - qw * qy)) - t0;

  // estimate tilt and roll offsets during first 100 samples
  if (imu_cnt < 100)
  {
    tsum += pitch;
    rsum += roll;
    imu_cnt++;
    if (imu_cnt == 100)
    {
      t0 = 0.01 * tsum;
      r0 = 0.01 * rsum;
    }
  }
}


//= Convert battery voltage to approximate capacity percentage.

void jhcWansuiAct::cb_batt (const std_msgs::Float32::ConstPtr& batt)
{
  double v100 = 12.0, v20 = 10.5, v10 = 10.2, v0 = 9.8;
  double v = batt->data;

  if (v >= v100)
    pct = 100.0;
  else if (v >= v20)
    pct = 80.0 * (v - v20) / (v100 - v20) + 20.0;
  else if (v >= v10)
    pct = 10.0 * (v - v10) / (v20 - v10) + 10.0;
  else if (v >= v0)
    pct = 10.0 * (v - v0) / (v10 - v0);
  else
    pct = 0.0;
}


///////////////////////////////////////////////////////////////////////////
//                                Neck                                   //
///////////////////////////////////////////////////////////////////////////

//= Set the gaze direction of the Orbbec camera and the graphical face.
// only tilt is physically actuated, pan happens by body rotation
// face shows any residual pan and half the tilt (+/- 60 looks bad)

void jhcWansuiAct::neck_issue ()
{
  hiwonder_servo_msgs::CommandDuration servo;
  geometry_msgs::Point face;
  double perr, nt;

  // get next incremental tilt angle command
  perr = alia_npt - (pan + cp0);
  nt = gaze_slew(nt2, ntc, alia_ntt - ct0, sf * alia_ntv, 60.0);

  // send pan and tilt command (if changed)
  if (nt != nt0)
  {
    servo.data = D2R * nt;
    servo.duration = (int)(1000.0 * lead * cyc + 0.5);
    neck_pub.publish(servo);
  }
  if ((perr != perr0) || (nt != nt0))
  {
    face.x = perr;             
    face.y = 0.5 * nt;
    face.z = sf * fmax(alia_npv, 0.5 * alia_ntv) * nsp;
    gaze_pub.publish(face);
  }

  // save commands for next cycle
  perr0 = perr;
  nt0 = nt;
}


//= Use velocity limited ramping to get current angle command.
// "n2" is ideal next point, "nc" is current angle, "nt" is goal
// assumes "nc" not updated yet, final angle clamped to +/- "lim" range

double jhcWansuiAct::gaze_slew (double& n2, double& nc, double nt, double rate, double lim) const
{
  double err, inc = cyc * nsp * rate;

  nc = n2;                   // assume perfect tracking
  err = nc - nt;
  if (err < 0.0)
    n2 = nc + fmin(inc, -err);
  else
    n2 = nc - fmin(inc, err);
  n2 = fmax(-lim, fmin(n2, lim));
  return(nc + lead * (n2 - nc));
}


//= Report the current Orbbec camera position and orientation.

void jhcWansuiAct::neck_update ()
{
  double trads = D2R * ntc;

  // current camera position (pan irrelevant)
  alia_nx = 0.0;
  alia_ny = (float)(ny - cup * sin(trads) + cfwd * cos(trads));
  alia_nz = (float)(nz + cup * cos(trads) + cfwd * sin(trads));

  // current gaze direction
  alia_np = (float)(pan + cp0);        // notional wrt travel dir
  alia_nt = (float)(ntc + ct0);
  alia_nr = (float) cr0;
}


///////////////////////////////////////////////////////////////////////////
//                                 Arm                                   //
///////////////////////////////////////////////////////////////////////////

//= Send new joint angles to arm servos based on goals.

void jhcWansuiAct::arm_issue ()
{
  double x = alia_axt, y = alia_ayt, z = alia_azt, t = alia_att; 
  double b, s, e, w, g, p, sp = sf * fmax(alia_apv, alia_adv);
  int tex = alia_adm & 0x02;

  // assert commands from ALIA
  arm.Grip(alia_awt, sf * alia_awv);
  if ((alia_aji > alia_api) && (alia_aji > alia_adi))
  {
    // tuck in arm
    arm.Home(b, s, e, w);
    arm.Pose(b, s, e, w, sf * alia_ajv);
  }
  else
  {
    // defaults for just orientation or just position change
    if (alia_apv == 0.0)
      arm.Position(x, y, z);
    if (alia_adv == 0.0)
    {
      arm.Orientation(p, t);
      tex = 0;                         // any tilt
    }

    // possibly set for linear instead of ang trajectory
    if ((alia_apm != 0) || ((alia_adm & 0x07) != 0))
      sp = -sp;
    arm.Move(x, y, z, t, tex, sp);
  }
  
  // find next incremental pose and send it
  // JetAuto Pro uses negated versions of b, w (s, e, g okay)
  arm.Ramp(b, s, e, w, g);
  arm_servos(-b, s, e, -w, g);
}


//= Set physical arm joint angles to specifed values (degs).

void jhcWansuiAct::arm_servos (double b, double s, double e, double w, double g)
{
  hiwonder_servo_msgs::CommandDuration servo;
  double br = D2R * b, sr = D2R * s, er = D2R * e, wr = D2R * w, gr = D2R * g;

  // send any changed angle to relevant servo (in radians)
  servo.duration = (int)(1000.0 * arm.lead * arm.cyc + 0.5);
  if ((servo.data = br) != br0)
    arm_bpub.publish(servo);
  if ((servo.data = sr) != sr0)
    arm_spub.publish(servo);
  if ((servo.data = er) != er0)
    arm_epub.publish(servo);
  if ((servo.data = wr) != wr0)
    arm_wpub.publish(servo);
  if ((servo.data = gr) != gr0)
    arm_gpub.publish(servo);

  // remember last angles sent (in radians)
  br0 = br;
  sr0 = sr;
  er0 = er;
  wr0 = wr;
  gr0 = gr;
}


//= Get new hand position, orientation, and width.
// assumes arm_issue() has already been called

void jhcWansuiAct::arm_update ()
{
  double x, y, z, p, t, b, s, e, w;

  // gripper position and orientation
  arm.Position(x, y, z);
  alia_ax = (float) x;
  alia_ay = (float) y;
  alia_az = (float) z;
  arm.Orientation(p, t);
  alia_ap = (float) p;
  alia_at = (float) t;
  alia_ar = 0.0;

  // finger separation and force
  alia_aw = (float) arm.Width();
  alia_af = (float) arm.Squeeze();

  // deviation from tucked pose
  arm.Home(b, s, e, w);
  alia_aj = (float) arm.ErrAng(b, s, e, w);
}


//= Get current servo angles from servo consolidator topic.
// assumes "names" = [joint1, joint2, joint3, joint4, n_joint, r_joint]
// sets joint angles in arm and also neck tilt

void jhcWansuiAct::servo_angs ()
{
  sensor_msgs::JointState::ConstPtr jts;
  double nr0;
  int i; 

  // keep reading messages until main arm angles are non-zero
  for (i = 0; i < 50; i++)
  {
    jts = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
    if ((jts->position[0] != 0.0) && (jts->position[1] != 0.0) &&
        (jts->position[2] != 0.0) && (jts->position[3] != 0.0))
      break;
  }

  // extract angles (in radians)
  br0 = jts->position[0];
  sr0 = jts->position[1];
  er0 = jts->position[2];
  wr0 = jts->position[3];
  nr0 = jts->position[4];
  gr0 = jts->position[5];

  // save as current pose for arm and neck (in degs)
  // JetAuto Pro uses negated versions of b, w (s, e, g okay)
  arm.InitPose(-R2D * br0, R2D * sr0, R2D * er0, -R2D * wr0, R2D * gr0);
  ntc = R2D * nr0;
  nt2 = ntc;
  nt0 = ntc;
}


///////////////////////////////////////////////////////////////////////////
//                                 Base                                  //
///////////////////////////////////////////////////////////////////////////

//= Send overall robot motion commands to wheels.
// Note: can only go about 7.5 ips sideways (rate = 0.4)

void jhcWansuiAct::base_issue ()
{
  geometry_msgs::Twist vels;
  double equiv, rrt, diag, rads = D2R * (alia_bsk - pan);

  // figure out combined rate for turning and panning
  equiv = alia_npv * nsp / rsp;
  if ((alia_brt * perr0) >= 0.0)       // from neck_issue         
    rrt = alia_brv + equiv;            
  else
    rrt = fabs(alia_brv - equiv);      // opposed directions

  // figure new ramped speeds based on ALIA commands
  alter_vel(mv, alia_bmt, sf * alia_bmv, msp, mup, mdn);
  alter_vel(rv, alia_brt + perr0, sf * rrt, rsp, rup, rdn);

  // only send message if velocities change
  if ((mv == mv0) && (rv == rv0) && (alia_bsk == sk0))
    return;
  mv0 = mv;
  rv0 = rv;
  sk0 = alia_bsk;

  // convert to m/s and skew drive direction
  diag = I2M * mv;                         
  vels.linear.x = diag * cos(rads);
  vels.linear.y = diag * sin(rads);
  vels.angular.z = D2R * rv;              
  base_pub.publish(vels);
}


//= Change velocity at rate "rt" to reduce "inc" remaining change.
// scales accelerations to give same trajectory regardless of rate
// makes sure that limited deceleration will cause stop at goal
// <pre>
//        ^
//     sp |       +-----------
//        |     /
//        |   /
//        | /
//       -+------------------->
//                       dist
// </pre>

void jhcWansuiAct::alter_vel (double& v, double inc, double rt, double vn, double tup, double tdn) 
{
  double vmax = rt * vn, acc = rt * vmax / tup, dec = rt * vmax / tdn;
  double vstop, dt = 1.0 / 30.0;

  // changes in speed are relative to goal direction
  if (inc < 0.0)
  {
    acc = -acc;
    dec = -dec;
  }

  // if going wrong way decelerate toward zero
  if ((inc * v) < 0.0)
    v += dec * dt;
  else
  {
    // accelerate (assuming far from goal)
    v += acc * dt;  

    // limit speed by goal deceleration
    vstop = sqrt(2.0 * dec * inc);
    if (vstop < vmax)
      vmax = vstop;
  }     

  // clip speed to valid range
  if (v > vmax)
    v = vmax;
  else if (v < -vmax)
    v = -vmax;
}


//= Get global Cartesian position and heading.

void jhcWansuiAct::base_update ()
{
  double dr, equiv, dp;

  // see if panning virtual neck  
  if (alia_npv > 0.0)
  {
    // find incremental rotation since last cycle
    dr = yaw - yaw0;
    if (dr > 180.0)
      dr -= 360.0;
    else if (dr <= -180.0)
      dr += 360.0;

    // adjust internal pan value by some fraction
    equiv = alia_npv * nsp / rsp;
    pan += dr * equiv / (alia_brv + equiv);
  }

  // communicate most recent Cartesian estimate
  alia_bx = xmap;
  alia_by = ymap;
  alia_bh = yaw - pan;
  yaw0 = yaw;
}


//= Stop all wheel rotation.

void jhcWansuiAct::base_stop ()
{
  geometry_msgs::Twist vels;

  // stop motor rotation
  vels.linear.x  = 0.0;
  vels.linear.y  = 0.0;
  vels.angular.z = 0.0;
  base_pub.publish(vels);

  // remember this command
  mv = 0.0;
  rv = 0.0;
  mv0 = 0.0;
  rv0 = 0.0;
  sk0 = 0.0;
}


//= Analyse fused IMU and wheel odometry to give Cartesian position.

void jhcWansuiAct::cb_odom (const nav_msgs::Odometry::ConstPtr& odom)
{
  xmap = M2I * odom->pose.pose.position.x;
  ymap = M2I * odom->pose.pose.position.y;
}

