// jhcWansuiVis.cpp : ALIA reasoner ROS node for JetAuto Pro with vision
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

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/xmlrpc_manager.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <hiwonder_servo_msgs/CommandDuration.h>
              
#include <alia_vis.h>                  

#include <jhcWansuiVis.h>


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
// Note: ROS read handler

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
// start with: roslaunch wansui_vis wansui_vis.launch show:=1

int main (int argc, char *argv[])
{
  ros_guard(argc, argv, "wansui_vis");
  jhcWansuiVis w;
  w.Reset();
  w.Run();
  return 0;
}


///////////////////////////////////////////////////////////////////////////
//                      Creation and Initialization                      //
///////////////////////////////////////////////////////////////////////////

//= Default destructor does necessary cleanup.

jhcWansuiVis::~jhcWansuiVis ()
{
  base_stop();
}


//= Default constructor initializes certain values.
// should call Reset() next, before Run()

jhcWansuiVis::jhcWansuiVis ()
{
  // whether to show debugging images 
  nh.param("show", show, 0);
  if (show > 0)
    show_init();

  // outgoing messages
  speak_pub = nh.advertise<std_msgs::String>("speak", 1);
  stare_pub = nh.advertise<std_msgs::Bool>("stare", 1);
  gaze_pub  = nh.advertise<geometry_msgs::Point>("gaze", 1); 
  mood_pub  = nh.advertise<std_msgs::Int32>("mood", 1);
  base_pub  = nh.advertise<geometry_msgs::Twist>("jetauto_controller/cmd_vel", 1);

  // outgoing servo messages
  pan_pub  = nh.advertise<hiwonder_servo_msgs::CommandDuration>("p_joint_controller/command_duration", 1);
  tilt_pub = nh.advertise<hiwonder_servo_msgs::CommandDuration>("t_joint_controller/command_duration", 1);
  arm_bpub = nh.advertise<hiwonder_servo_msgs::CommandDuration>( "joint1_controller/command_duration", 1);
  arm_spub = nh.advertise<hiwonder_servo_msgs::CommandDuration>( "joint2_controller/command_duration", 1);
  arm_epub = nh.advertise<hiwonder_servo_msgs::CommandDuration>( "joint3_controller/command_duration", 1);
  arm_wpub = nh.advertise<hiwonder_servo_msgs::CommandDuration>( "joint4_controller/command_duration", 1);
  arm_gpub = nh.advertise<hiwonder_servo_msgs::CommandDuration>("r_joint_controller/command_duration", 1);

  // incoming messages ("joint_states" handled by polling)
  talk_sub = nh.subscribe<std_msgs::Bool>("talking", 10, &jhcWansuiVis::cb_talk, this);
  imu_sub  = nh.subscribe<sensor_msgs::Imu>("imu", 10, &jhcWansuiVis::cb_imu, this);
  odom_sub = nh.subscribe<nav_msgs::Odometry>("odom_raw", 10, &jhcWansuiVis::cb_odom, this);
  batt_sub = nh.subscribe<std_msgs::Float32>("voltage", 10, &jhcWansuiVis::cb_batt, this);
  mic_sub  = nh.subscribe<std_msgs::Int32>("sound_dir", 10, &jhcWansuiVis::cb_mic, this);
  col_sub  = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw",         // faster
                                              10, &jhcWansuiVis::cb_color, this);    
  rng_sub  = nh.subscribe<sensor_msgs::Image>("/camera/depth_registered/image_raw", 
                                              10, &jhcWansuiVis::cb_range, this);

  // neck camera geometry
  ny   = -6.30;              // pan in front of wheel center (160mm)
  nz   = 20.47;              // pan axis height above floor (520mm)
  clf  =  0.47;              // camera to left of midline (12mm)
  cfwd =  0.79;              // camera in front of tilt axis (20mm)
  cup  =  3.20;              // camera above tilt axis (81mm)

  // neck control
  nsp  = 90.0;               // pan and tilt speed (dps)
  lead =  1.0;               // servo response time vs cyc

  // base motion ramping  
  msp =  18.0;               // full move speed (ips)
  mup =   1.0;               // secs to full move
  mdn =   1.0;               // secs to move stop
  rsp = 120.0;               // full turn speed (dps)
  rup =   1.0;               // secs to full turn
  rdn =   1.0;               // secs to turn stop 

  // whether any depth images have arrived
  rcnt = 0;
}


//= Create and display token debugging images.
// call as early as possible so face windows go over top

void jhcWansuiVis::show_init ()
{
  // object detection 
  cam = cv::Mat(480, 640, CV_8UC3, cv::Scalar(255, 0, 0)); 
  cv::namedWindow("Camera View");
  cv::moveWindow("Camera View", 0, 0);
  cv::imshow("Camera View", cam);

  // navigation obstacles (or debugging image - show last so on top)
  map = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 128, 0)); 
  cv::namedWindow("Overhead Map");
  cv::moveWindow("Overhead Map", 680, 0);
  cv::imshow("Overhead Map", map);
  mtitle = 0;

  // needs >200ms for fill
  cv::waitKey(500);                  
}


//= Load servo calibration file for this specific body.
// lines have form "boff: -3.5" (YAML)

void jhcWansuiVis::calibrate (const char *dir, const char *rname)
{
  char key[14][20] = {"bsc:",  "ssc:",  "esc:",  "wsc:",  "psc:",  "tsc:",  "gsc:",  
                      "boff:", "soff:", "eoff:", "woff:", "poff:", "toff:", "goff:"};
  double *param[14] = {&bsc,  &ssc,  &esc,  &wsc,  &psc,  &tsc,  &gsc, 
                       &boff, &soff, &eoff, &woff, &poff, &toff, &goff};
  char txt[80], tag[20];
  FILE *in;
  double val;
  int i;

  // set default scaling and offsets (skip base and grip)
  for (i = 1; i < 6; i++)
    *(param[i]) = 1.0;       
  for (i = 7; i < 13; i++)
    *(param[i]) = 0.0;

  // JetAuto exceptions
  bsc = -1.0;                // base and wrist are reversed
  wsc = -1.0;
  gsc =  1.3;                // gripper 240 vs 180 deg
  goff = -40.0;

  // try opening file
  sprintf(txt, "%sconfig/%s_servo.yaml", dir, rname);
  if ((in = fopen(txt, "r")) == NULL)
    return;

  // read tagged values
  while (fgets(txt, 80, in) != NULL)
    if (sscanf(txt, "%s %lf", tag, &val) == 2)
      for (i = 0; i < 14; i++)
        if (strcmp(tag, key[i]) == 0)
        {
          *(param[i]) = val;
          break;
        }

  // cleanup
  fclose(in);
}


//= Initialize all variables for start of next run.

void jhcWansuiVis::Reset ()
{
  // arm parameters (angles copied from init_pose.py)
  arm.Reset();
  g0 = arm.Angles(b0, s0, e0, w0);

  // neck angles (copied from init_pose.py)
  npc = 0.0;
  ntc = 0.0;
  np2 = npc;
  nt2 = ntc;
  np0 = npc;
  nt0 = ntc;

  // IMU zero auto-calibration
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
  volts = 12.0;
  pct   = 100.0; 
  sdir  = 0;

  // speed factor and previous mood bits
  sf = 1.0;
  mood0 = 0;

  // base previous location and heading
  xpos = 0.0;
  ypos = 0.0;
  yaw  = 0.0;

  // base motion and position
  trav = 0.0;
  wind = 0.0;
  xmap = 0.0;
  ymap = 0.0;

  // previous and current base speeds
  mv0 = 0.0;
  rv0 = 0.0;
  sk0 = 0.0;
  mv  = 0.0;
  rv  = 0.0;

  // last command time
  tick = 0.0;
  cyc = 0.0333;
}


///////////////////////////////////////////////////////////////////////////
//                               Main Loop                               //
///////////////////////////////////////////////////////////////////////////

//= Enter reasoning loop with user interaction and robot body.
// should call Reset() before this

void jhcWansuiVis::Run ()
{
  ros::Rate rate(30.0);
  start();
  while (ros::ok() && (wansui_kill <= 0))
  {  
    ros::spinOnce();         // allow callbacks to run
    update();
    if (alia_think() <= 0)
      break;
    issue();
    rate.sleep();
  }
  shutdown();                // called even for kill
}


//= Configure and start up all components.

void jhcWansuiVis::start ()
{
  char rname[80], dir[80];
  int rc, wait = 0;

  // stop any overall motion
  rc = system("sudo pulseaudio -k");   // optional
  base_stop();  

  // wait for IMU and graphical head (needs 4 sec)
  ros::Rate rate(30.0);
  while ((imu_cnt <= 0) || (wait++ < 120))  
  {
    if (!ros::ok() || (wansui_kill > 0))
      return;
    ros::spinOnce();         // callback increments imu_cnt
    rate.sleep();
  }

  // reboot USB hub if range images not streaming by now 
  if (rcnt < 100)
  {
    printf("\x1b[31;1m[wansui_vis] Have to reboot Orbbec depth sensor -- try again!\x1b[0m\n"); 
    rc = system("aplay -q /home/jetauto/Wansui/sfx/squawk.wav");
    rc = system("sudo uhubctl -l 1-2 -a off");
    sleep(1);
    rc = system("sudo uhubctl -l 1-2 -a on");
    sleep(3);
    wansui_kill = 1;
    return;
  } 

  // load calibration for arm and neck servos
  sprintf(dir, "%s/Wansui/", getenv("HOME"));
  gethostname(rname, 80);
  calibrate(dir, rname);   
  
  // start reasoner with working directory, robot name, and debugging image
  strcat(rname, " Wansui");
  alia_reset(dir, rname, "wansui_vis", show);

  // start speech recognizer (needs all_names.txt from running reasoner)
  rc = system("pactl set-source-mute @DEFAULT_SOURCE@ 0"); 
  if (reco.Start(dir, 0) <= 0)
  {
    rc = system("aplay -q /home/jetauto/Wansui/sfx/toot.wav &");
    printf("\x1b[33;1m[wansui_vis] Could not start speech recognition!\x1b[0m\n"); 
  } 

  // possibly request debugging images (alia_reset zeroes these vals)
  if (show > 0)
  {    
    if ((map.cols != alia_wmap()) || (map.rows != alia_hmap()))
    { 
      map.release();                   // wrong size
      map = cv::Mat(alia_hmap(), alia_wmap(), CV_8UC3, cv::Scalar(0, 128, 0));
      cv::moveWindow("Overhead Map", 680, 0);
    } 
    if (mtitle <= 0)                   
    { 
      cv::setWindowTitle("Overhead Map", alia_tmap());     // re-title on first call
      mtitle = 1;
    }

    alia_view = cam.data;
    alia_map = map.data;
    alia_vfmt = 2;
    alia_mfmt = 2;
  }  
}
 

//= Get data from sensors and transfer to ALIA reasoner.

void jhcWansuiVis::update ()
{
  // gather all sensor inputs
  reco_update();
  body_update();
  neck_update();
  arm_update();
  base_update();
}


//= Transfer commands from ALIA reasoner to actuators.

void jhcWansuiVis::issue ()
{
  timespec ts;
  double dt, t0 = tick;

  // show debugging images
  if (show > 0)
    status_imgs();

  // get true time since last command (fairly uneven!)
  clock_gettime(CLOCK_BOOTTIME, &ts);
  tick = (double) ts.tv_sec + 1.0e-9 * ts.tv_nsec;
  dt = ((t0 <= 0.0) ? 0.0333 : tick - t0);
  cyc += 0.05 * (dt - cyc);

  // send all command outputs
  tts_issue();
  body_issue();
  neck_issue();
  arm_issue();
  base_issue();
}  


//= Cleanly stop all actions and save data.

void jhcWansuiVis::shutdown ()
{
  int rc;

  alia_done(1);
  base_stop();
  cv::destroyAllWindows();
  rc = system("pactl set-source-mute @DEFAULT_SOURCE@ 0"); 
  printf("Done (%4.2fV)\n\n", volts);
  ros::shutdown(); 
}


///////////////////////////////////////////////////////////////////////////
//                                Speech                                 //
///////////////////////////////////////////////////////////////////////////

//= Possibly speak output text or change eye center color. 

void jhcWansuiVis:: tts_issue ()
{
  std_msgs::String msg;
  const char *output;
 
  output = alia_spout();
  if (*output != '\0')
  {
    msg.data = output;                 // copies string
    speak_pub.publish(msg);            // ROS send
  }
}


//= Get any speech recognition results and set status flags.

void jhcWansuiVis::reco_update ()
{
  char text[500];

  alia_hear = reco.Status();
  if (alia_hear == 2)
    alia_spin(reco.Heard(text), reco.Delay());
  alia_talk = talking;
}


//= Mute audio input when talking and remember state.
// Note: ROS read handler

void jhcWansuiVis::cb_talk (const std_msgs::Bool::ConstPtr& busy)
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

void jhcWansuiVis::body_issue ()
{
  std_msgs::Bool eye;
  std_msgs::Int32 m;

  // turn eyes blue when listening (no need to say name)
  if (alia_attn != attention)
  {
    eye.data = ((alia_attn > 1) ? true : false);
    stare_pub.publish(eye);                                // ROS send
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

    // modulate voice and face based on mood bits
    m.data = alia_mood;
    mood_pub.publish(m);               // ROS send
    mood0 = alia_mood;
  }
}


//= Get battery capacity and body attitude.

void jhcWansuiVis::body_update () const
{
  alia_batt = pct;
  alia_tilt = pitch;
  alia_roll = roll;
  alia_snd  = sdir;
}


//= Analyse IMU readings to give heading and body attitude.
// Note: ROS read handler

void jhcWansuiVis::cb_imu (const sensor_msgs::Imu::ConstPtr& imu)
{
  double qx, qy, qz, qw, qx2, qy2, qz2, qw2, dh, yaw0 = yaw; 

  // get elements of quaternion
  qx = imu->orientation.x;
  qy = imu->orientation.y;
  qz = imu->orientation.z;
  qw = imu->orientation.w;
  qx2 = qx * qx;
  qy2 = qy * qy;
  qz2 = qz * qz;
  qw2 = qw * qw;

  // resolve into yaw, pitch, and roll (Tait-Bryan yaw->pitch->roll)
  // Wikipedia "Rotation formalisms in three dimensions"
  // section "Quaternion -> Euler angles (z-y′-x″ intrinsic)"
  // uses identity: qx2 + qy2 + qz2 + qw2 = 1
  yaw   =  R2D * atan2(2.0 * (qx * qy + qw * qz), qw2 + qx2 - qy2 - qz2);
  roll  =  R2D * atan2(2.0 * (qy * qz + qw * qx), qw2 - qx2 - qy2 + qz2) - r0;
  pitch = -R2D * asin( 2.0 * (qx * qz - qw * qy)) - t0;

  // get cumulative turning
  dh = yaw - yaw0;
  if (dh > 180.0)
    dh -= 360.0;
  else if (dh <= -180.0)
    dh += 360.0;
  if (fabs(dh) > 0.1)                  // squelch under 3 deg/sec
    wind += dh;

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
// Note: ROS read handler

void jhcWansuiVis::cb_batt (const std_msgs::Float32::ConstPtr& batt)
{
  double v100 = 11.7, v10 = 9.9, v0 = 9.6;

  volts = batt->data;
  if (volts >= v100)
    pct = 100.0;
  else if (volts >= v10)
    pct = 90.0 * (volts - v10) / (v100 - v10) + 10.0;
  else if (volts >= v0)
    pct = 10.0 * (volts - v0) / (v10 - v0);
  else
    pct = 0.0;
}


//= Remember direction of arrival for sound.
// mic array mounted on nose at 45 degrees
// Note: ROS read handler

void jhcWansuiVis::cb_mic (const std_msgs::Int32::ConstPtr& dir)
{
  sdir = dir->data;
}


///////////////////////////////////////////////////////////////////////////
//                                Neck                                   //
///////////////////////////////////////////////////////////////////////////

//= Set the gaze direction of the Orbbec camera and the graphical face.
//   p = pan degs CCW from forward (+45 = angled left)
//   t = tilt degs wrt horizontal  (-45 = angled down)
// face shows any residual pan and half the tilt (+/- 60 looks bad)
// NOTE: current pan and tilt angles updated by arm_update() 

void jhcWansuiVis::neck_issue ()
{
  hiwonder_servo_msgs::CommandDuration servo;
  geometry_msgs::Point face;
  double dx, dy, dz, dr, r, pan, tilt, np, nt;

  // check if given target angles as opposed to xyz location
  if ((alia_rpi >= alia_rgi) || (alia_rti >= alia_rgi))
  {
    // get next incremental pan and tilt angle commands
    np = gaze_slew(np2, npc, alia_rpt, sf * alia_rpv, 60.0);
    nt = gaze_slew(nt2, ntc, alia_rtt, sf * alia_rtv, 60.0);
  }
  else
  {
    // compute approximate pan goal (ignores clf, cfwd, cp0 calib)
    dx = alia_rxt;
    dy = alia_ryt - ny;
    dz = alia_rzt - nz;
    dr = sqrt(dx * dx + dy * dy);
    pan = R2D * atan2(-dx, dy);

    // compute approximate tilt goal (ignores clf, cfwd, ct0 calib)
    //   tr = dr + cup * sin(t)
    //   tz = dz - cup * cos(t)
    //       tan(t) = sin(t) / cos(t) = tz / tr
    //                    tr * sin(t) = tz * cos(t)
    //   dr * sin(t) + cup * sin^2(t) = dz * cos(t) - cup * cos^2(t)
    //                            cup = dz * cos(t) - dr * sin(t) 
    //
    // let r = sqrt(dz^2 + dr^2), cos(k) = dz / r, sin(k) = dr / r
    //     cup = [r * cos(k)] * cos(t) - [r * sin(k)] * sin(t)
    //     cup = r * cos(t + k)  
    //   t + k = acos(up / r)
    r = sqrt(dr * dr + dz * dz);
    tilt = R2D * (acos(cup / r) - acos(dz / r));
   
    // get next incremental pan and tilt angle commands
    np = gaze_slew(np2, npc, pan,  sf * alia_rgv, 60.0);
    nt = gaze_slew(nt2, ntc, tilt, sf * alia_rgv, 60.0);
  }

  // send pan and tilt commands (if changed)
  servo.duration = (int)(1000.0 * lead * cyc + 0.5);
  if (np != np0)
  {
    servo.data = D2R * (psc * np + poff);
    pan_pub.publish(servo);                                // ROS send
  }
  if (nt != nt0)
  {
    servo.data = D2R * (tsc * nt + toff);
    tilt_pub.publish(servo);                               // ROS send
  }

  // change graphical face orientation
  if ((np != np0) || (nt != nt0))
  {
    face.x = 0.5 * np;             
    face.y = 0.5 * nt;
    face.z = 0.5 * sf * fmax(alia_rpv, alia_rtv) * nsp;
    gaze_pub.publish(face);                                // ROS send
  }

  // remember last perfect angles sent (in degs)
  np0 = np;
  nt0 = nt;
}


//= Use velocity limited ramping to get current angle command.
// "n2" is ideal next point, "nc" is current angle, "nt" is goal
// assumes "nc" not updated yet, final angle clamped to +/- "lim" range

double jhcWansuiVis::gaze_slew (double& n2, double& nc, double nt, double rate, double lim) const
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

void jhcWansuiVis::neck_update () const
{
  double prads = D2R * npc, cosp = cos(prads), sinp = sin(prads);
  double trads = D2R * ntc, cost = cos(trads), sint = sin(trads);
  double r = -cup * sint + cfwd * cost;          // horizontal to midline

  // current camera position (Y is forward, X is to right)
  alia_rx = (float)(-r * sinp - clf * cosp);
  alia_ry = (float)( r * cosp - clf * sinp + ny);
  alia_rz = (float)(cup * cost + cfwd * sint + nz);
  alia_cx = alia_rx;
  alia_cy = alia_ry;
  alia_cz = alia_rz;
  alia_nx = alia_rx;
  alia_ny = alia_ry;
  alia_nz = alia_rz;

  // current gaze direction (camera angular offsets fixed by alia_vis)
  alia_rp = (float) npc;  
  alia_rt = (float) ntc;         
  alia_rr = 0.0;
  alia_cp = alia_rp;
  alia_ct = alia_rt;
  alia_cr = alia_rr;
  alia_np = alia_rp;
  alia_nt = alia_rt;
  alia_nr = alia_rr;
}


///////////////////////////////////////////////////////////////////////////
//                                 Arm                                   //
///////////////////////////////////////////////////////////////////////////

//= Send new joint angles to arm servos based on goals.

void jhcWansuiVis::arm_issue ()
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
//    if ((alia_apm != 0) || ((alia_adm & 0x07) != 0))
//      sp = -sp;
    arm.Move(x, y, z, t, tex, sp);
  }
  
  // find next incremental pose and send it
  arm.Ramp(b, s, e, w, g, cyc);
  arm_servos(b, s, e, w, g);
}


//= Set physical arm joint angles to specifed values (degs).
// adds in calibration offsets, fixes gripper scaling 
// canonical arm angles:
//   b = base rotation CCW from front        (+45 = angled left)
//   s = shoulder degs forward from vertical (+45 = forward up)
//   e = elbow degs down from shoulder link  (+45 = angled down)
//   w = wrist degs up from elbow link       (-45 = angled down)
//   g = gripper left link degs wrt midline  (+90 = full open)
// NOTE: neck servo commands published separately by neck_issue()

void jhcWansuiVis::arm_servos (double b, double s, double e, double w, double g)
{
  hiwonder_servo_msgs::CommandDuration servo;

  // send any changed angle to relevant servo (in radians)
  servo.duration = (int)(1000.0 * arm.lead * cyc + 0.5);
  if (b != b0)
  {
    servo.data = D2R * (bsc * b + boff);
    arm_bpub.publish(servo);                               // ROS send
  }
  if (s != s0)
  {
    servo.data = D2R * (ssc * s + soff);                 
    arm_spub.publish(servo);                               // ROS send
  }
  if (e != e0)
  {
    servo.data = D2R * (esc * e + eoff);
    arm_epub.publish(servo);                               // ROS send
  }
  if (w != w0)
  {
    servo.data = D2R * (wsc * w + woff);
    arm_wpub.publish(servo);                               // ROS send
  }
  if (g != g0)
  {
    servo.data = D2R * (gsc * g + goff);       
    arm_gpub.publish(servo);                               // ROS send
  }

  // remember last perfect angles sent (in degs)
  b0 = b;
  s0 = s;
  e0 = e;
  w0 = w;
  g0 = g;
}


//= Get new hand position, orientation, and width.
// assumes arm_issue() has already been called

void jhcWansuiVis::arm_update () 
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


///////////////////////////////////////////////////////////////////////////
//                                 Base                                  //
///////////////////////////////////////////////////////////////////////////

//= Send overall robot motion commands to wheels.
// Note: can only go about 7.5 ips sideways (rate = 0.4)

void jhcWansuiVis::base_issue ()
{
  geometry_msgs::Twist vels;
  double diag, rads = D2R * alia_bsk;

  // figure new ramped speeds based on ALIA commands
  alter_vel(mv, alia_bmt - trav, sf * alia_bmv, msp, mup, mdn);
  alter_vel(rv, alia_brt - wind, sf * alia_brv, rsp, rup, rdn);

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
  base_pub.publish(vels);                        // ROS send
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

void jhcWansuiVis::alter_vel (double& v, double inc, double rt, double vn, double tup, double tdn) const
{
  double vstop, vmax = rt * vn, acc = rt * vmax / tup, dec = rt * vmax / tdn;

  // changes in speed are relative to goal direction
  if (inc < 0.0)
  {
    acc = -acc;
    dec = -dec;
  }

  // if going wrong way decelerate toward zero
  if ((inc * v) < 0.0)
    v += dec * cyc;
  else
  {
    // accelerate (assuming far from goal)
    v += acc * cyc;  

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

void jhcWansuiVis::base_update ()
{
  alia_bt = trav;
  alia_bw = wind;
  alia_bx = xmap;
  alia_by = ymap;
}


//= Stop all wheel rotation.

void jhcWansuiVis::base_stop ()
{
  geometry_msgs::Twist vels;

  // stop motor rotation
  vels.linear.x  = 0.0;
  vels.linear.y  = 0.0;
  vels.angular.z = 0.0;
  base_pub.publish(vels);                        // ROS send

  // remember this command
  mv = 0.0;
  rv = 0.0;
  mv0 = 0.0;
  rv0 = 0.0;
  sk0 = 0.0;
}


//= Analyse fused IMU and wheel odometry to give Cartesian position.
// Note: ROS read handler

void jhcWansuiVis::cb_odom (const nav_msgs::Odometry::ConstPtr& odom)
{
  double dx, dy, dist, x0 = xpos, y0 = ypos, rads = D2R * (yaw + sk0); 

  // get displacement (rotation inaccurate + plain "odom" glitches!)
  xpos = M2I * odom->pose.pose.position.x;
  ypos = M2I * odom->pose.pose.position.y;
  dx = xpos - x0;
  dy = ypos - y0;
  dist = sqrt(dx * dx + dy * dy);    

  // adjust accumulated travel and map position
  if (mv0 < 0.0)
    dist = -dist;
  trav += dist;
  xmap += dist * cos(rads);
  ymap += dist * sin(rads);
}


///////////////////////////////////////////////////////////////////////////
//                                Vision                                 //
///////////////////////////////////////////////////////////////////////////

//= Show detected objects and local obstacles.

void jhcWansuiVis::status_imgs ()
{
  cv::imshow("Camera View", cam);      // underneath in overlap area
  cv::imshow("Overhead Map", map);     
  cv::waitKey(1);                      // needed to pump update message
}


//= Remember most recent color image received and update capture status.
// Note: ROS read handler

void jhcWansuiVis::cb_color (const sensor_msgs::Image::ConstPtr& img)
{
  alia_col = (img->data).data();       
  alia_cfmt = 2;                       // data ready
  ckeep = img;                         // prevent garbage collection
}


//= Remember most recent depth image received and update capture status.
// Note: ROS read handler

void jhcWansuiVis::cb_range (const sensor_msgs::Image::ConstPtr& img) 
{
  rcnt++;                              // liveness count
  alia_rng = (img->data).data();       
  alia_rfmt = 2;                       // data ready
  rkeep = img;                         // prevent garbage collection
}

