# Calibration

## Arm & Neck Servos

The midpoint and scaling of the bus servos can vary from one unit to another. Getting accurate values for these parameters is crucial for grasping objects. From the Wansui directory run the command below and follow its instructions to move the arm into various poses. 

    ./ja_servo_cal

To further refine the calibration, follow up by running the related command below. This tries to optimize the arm pose for a typical grasping location.

    ./ja_grab_cal

Both these utilities will automatically write new values to the configuration file [Benny_servo](../config/Benny_servo.yaml) (or whatever your robot's name is). 

## Range Sensor

Accurate pointing of the Orbbec sensor is needed for proper hand-eye coordination and is controlled by the [Benny_cam](../config/Benny_cam.cal) file (or whatever your robot's name is). The file has a single line beginning with "grok_cal" whose fields are as shown below. The first 3 values are for the range image while the second 3 values are for the color image. You must edit this file manually.

    grok_cal  rng_pan rng_tilt rng_roll  col_pan col_tilt col_roll

To calibrate the range sensor, first place the robot in environment where it has a large expanse of floor in front of it (like 4 feet). Then fire up the complete system in debug mode using the command below, and enter the ALIA request shown. Now manually copy the displayed "dr" value to the third number in the calibration file. This corresponds to the __roll__ of the sensor.

    demo 3
    > look down

The next calibration step needs to be repeated any time the neck servo parameters (above) get changed. Place a small block directly in front of the robot so that its _center_ is 6" in front of the middle of the bumper. Run the command below and enter the subsequent ALIA request. You want the displayed object position to read (0.0 11.8) within +/- 0.2 inches. 

    demo 4
    > look far down

Doing this requires editing more configuration values. The first number after "grok_cal" affects the __pan__ of the sensor – a higher value here makes the displayed x coordinate _smaller_. Similarly, the second value changes the __tilt__ compensation – a higher value for this parameter makes the y coordinate _larger_.  Hit ESC to exit ALIA, change the values, then re-run the procedure until the reported object position becomes reasonably accurate. 

## Color Camera 

The Orbbec sensor generally registers its range image directly to its color image, however sometimes there seems to be a small vertical displacement. Start by __copying__ the first 3 value in the "grok_cal" line to the last 3 values (perfect registration). Now invoke demo mode and enter the request given below. The right hand image will now show a mask where the portion of the color image corresponding to the nearest object shows through. 

    demo 6
    > look down

Place an object in front of the robot and examine the object bounding box in the left hand image. Increasing the 5th value (col_tilt) moves the box _down_. As before, iterate between running the system and editing the file until the box is nicely centered on the object. At this point the mask on the right hand side should show only the object, no table fragments above or below it.

---

May 2026 - Jonathan Connell - jconnell@alum.mit.edu