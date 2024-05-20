## Software Installation

The easiest approach is just to copy a complete system image onto a new [SD card](https://www.amazon.com/dp/B0B7NVMBPL) (32/64GB microSDHX UHS-1). Note that this requires some disassembly of the JetAuto robot to get at the card slot. Contact me if you are interested in access to the image. You might want to upgrade to 64GB anyhow, as most of the original 32GB is filled up. Otherwise, there is a substantial list of downloads, substitutions, and configurations as listed below. Eventually there should be an installer script for most of this ...

### Switch to LXDE (optional)

JetAuto comes with Gnome as the desktop environment, but this is a real hog for a headless machine. Switching to LXDE frees up a bunch of resources. If you want to do this, the steps are:

1. logout using gear icon in upper right corner of desktop
2. on login screen click gear and select LXDE then login
3. at terminal: __sudo dpkg-reconfigure lightdm__
4. reboot (power button icon at lower right then select reboot)

### Additions and Substitutions

Start by copying over the entire Git directory to /home/jetauto/Wansui. This is the [ALIA](https://github.com/jconnell11/ALIA) reasoning library, a bunch of configuration files of various types, and some supporting ROS nodes. In particular it contains subprojects for the main ROS control node ([wansui_act](../wansui_act)) and the animated face ([hmore_face](../hmore_face)). These directory should be moved under ~/jetauto_ws/src/etaoin_sys since there can only be one active ROS workspace.

Next, the directory [jet_files](../jet_files) contains a number of files to add or substitute for similarly named versions in various locations. You should copy the top level files (e.g. .bashrc) to /home/jetauto. However, be sure to edit [.typerc](../jet_files/.typerc) to include the network name of your machine (instead of "Benny"). All the other subdirectories are condensed versions of the full pathnames where things belong. Note that the items in [usr_local_bin](../jet_files/usr_local_bin) and [etc_systemd_system](../jet_files/etc_systemd_system) will need to be marked as executable before being moved to /usr/local/bin and /etc/systemd/system, respectively.

### Configuration

Configure display for full screen animated face:

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
    catkin build wansui_act

Automatic fan speed control (optional):

1. download all from https://github.com/Pyrestone/jetson-fan-ctl
2. unzip jetson-fan-ctl-master.zip .
3. jetson-fan-ctl-master/install.sh




### Microsoft Azure

The system is default coded to use Microsoft Azure speech recognition, which is essentially __free__ for low intensity usage. However, you will need credentials to access this on-line service. Start by signing up [here](https://portal.azure.com/#create/Microsoft.CognitiveServicesSpeechServices) (possibly making a Microsoft account first) then select "Speech Services" and "+ Create". Finally, click "Manage keys" and modify local file [ms_azure.key](../wansui_act/config/ms_azure.key) in directory ~/jetauto_ws/src/etaoin_sys/wansui_act/config with valid "Key" and "Location" strings.

---

May 2024 - Jonathan Connell - jconnell@alum.mit.edu


