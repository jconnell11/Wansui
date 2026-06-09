## Software Installation

It is recommended that you switch the robot over to a 64GB Sandisk Extreme SD card. The __A2__ rating results in a faster boot and smoother audio, and most of the original 32GB (on the Hiwonder card) was filled up. This new card can be most conveniently programmed using the [Win32DiskImager](https://win32diskimager.org/) tool. Note that accessing the card slot, located under the front edge of the Jetson Nano mezzanine board, requires some drastic disassembly of the JetAuto robot (remove the arm)!

If you are going to flash a new card anyhow, the easiest option is to download a __pre-built__ image with all the necessary software. [Contact](mailto:jconnell@alum.mit.edu) me to access the download (free). Otherwise, start by using Win32DiskImager to copy the original Hiwonder card to an image file, then burn the image file back to the new 64GB card. After this there are a substantial number of downloads, substitutions, and configurations as listed below.

### Switch to LXDE (optional)

JetAuto comes with Gnome as the desktop environment, but this is a real hog for a headless machine. Switching to LXDE frees up a lot of resources. If you want to do this, the steps are:

1. logout using gear icon in upper right corner of desktop
2. on login screen click gear and select LXDE then login
3. at terminal: __sudo dpkg-reconfigure lightdm__
4. reboot (power button icon at lower right then select reboot)

### Additions and Substitutions

Start by copying over the entire Git directory to /home/jetauto/Wansui. This is the [ALIA](https://github.com/jconnell11/ALIA) reasoning library, a bunch of configuration files of various types, and some supporting ROS nodes. In particular, it contains subprojects for the main ROS control node ([wansui_vis](../project/wansui_vis)) and the animated face ([hmore_face](../project/hmore_face)). These directories should be moved under ~/jetauto_ws/src/etaoin_sys since there can only be one active ROS workspace.

Next, the directory [jet_files](../project/jet_files) contains a number of files to add or substitute for similarly named versions in various locations. The subdirectories are condensed versions of the full pathnames where things belong. However, all the items in [usr_local_bin](../project/jet_files/usr_local_bin) will need to be marked as executable before being moved to /usr/local/bin. Also be sure to edit [.typerc](../project/jet_files/home_jetauto/.typerc) to include the network name of your machine (instead of "Benny"). Finally, modify [hiwonder_wifi_conf](../project/jet_files/home_jetauto/hiwonder-toolbox/hiwonder_wifi_conf.py) and [alt_wifi](../project/jet_files/home_jetauto/hiwonder-toolbox/alt_wifi.py) to reflect your main wifi connection credentials and a backup network (if any).

### Configuration

Set up display for full screen animated face:

1. select hidden files in file browser (from triple dashes)
2. edit /home/jetauto/.display.sh and add: __xrandr -o inverted__
3. at terminal: __xscreensaver-demo__
4. select Mode: Disable Screen Saver
5. hide LXDE taskbar: Panel / Advanced / "Minimize panel when not in use"

Substitute new code for buttons on expansion board:

    sudo systemctl disable start_app_node.service
    sudo systemctl disable voltage_detect.service
    sudo systemctl enable ja_buttons.service

Install TTS software and some system tools:

    sudo apt install festival festival-dev libasound2 soundstretch screen

Compile new ROS nodes:

    cd ~/jetauto_ws
    catkin build hmore_face
    catkin build wansui_vis

Automatic fan speed control (optional):

1. download all from https://github.com/Pyrestone/jetson-fan-ctl
2. unzip jetson-fan-ctl-master.zip .
3. jetson-fan-ctl-master/install.sh


### Microsoft Azure

The system is default coded to use Microsoft Azure speech recognition, which is essentially __free__ for low intensity usage. However, you will need credentials to access this on-line service. Start by signing up [here](https://portal.azure.com/#create/Microsoft.CognitiveServicesSpeechServices) (possibly making a Microsoft account first) then select "Speech Services" and "+ Create". Finally, click "Manage keys" and modify local file [ms_azure.key](../project/config/ms_azure.key) in directory ~/Wansui with valid "Key" and "Location" strings.

---

May 2026 - Jonathan Connell - jconnell@alum.mit.edu


