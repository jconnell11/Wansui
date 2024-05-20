# Wansui
## Mobile Manipulator with Face

Add verbal interaction and symbolic learning to a relatively inexpensive mobile robot with an arm and 3D sensing! This robot is a somewhat modified version of the commercially available [JetAuto Pro](https://www.hiwonder.com/collections/ros-robot/products/jetauto-pro?variant=40040875229271) (Standard) from Hiwonder and can be assembled for under $1400 (far cheaper than [LoCoBot](https://www.generationrobots.com/en/404057-locobot-autonomous-mobile-robot.html)). The system is based on the [ALIA](https://github.com/jconnell11/ALIA) library and runs with ROS on a Jetson Nano under Ubuntu. It also sports an animated [face](https://github.com/jconnell11/hmore_face), as shown in this [video](https://youtu.be/DcIPQSiz_0I).

![Benny robot](Benny.jpg)

### Getting Started

The robot needs various [hardware](doc/hardware.md) modifications including moving the screen, adding a neck servo, and the installation of a sound card. After this, there are a number of [software](doc/software.md) changes that should be performed. The basic robot with Jetson Nano costs about $1280 (with shipping), while the extensions add another $75.

When all the modifications are complete, you should be able to invoke the demo by a short press of the __centermost__ button near the back right corner of the circuitboard. Use a short press of the other, right button near the edge of the board to exit. A long press of the right button will cleanly shut down the robot, but you must still power it off manually (after the blue screen). Note, you will need to say "robot" or the network name of the machine (e.g. "Benny") to get the robot's attention (eyes turn light blue). 

You can also type to the robot if you prefer that to speaking. The easiest way is to connect via NoMachine and type the command "demo" (or alternatively "cd ~/Wansui" then "roslaunch wansui_act wansui_act.launch"). You will have to resurface the command window (e.g. Alt-Tab) on top of the face in order to type. If you instead started the robot using the button, you can use the command "connect" (e.g. from a remote SSH session). No attention word is necessary when typing.

Right now the robot can move on command (e.g. "turn right" or "gaze up" or "extend the arm"). In general, it can perform all the sorts of learning shown in this [video](https://youtu.be/EjzdjWy3SKM). However, it cannot navigate, or find and grab objects yet. This will be possible eventually ... 

For an even cheaper version see [Ganbei](https://github.com/jconnell11/Ganbei).

---

May 2024 - Jonathan Connell - jconnell@alum.mit.edu


