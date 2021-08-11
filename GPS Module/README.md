The ROS setup for GPS module (firmware version 9) ([SparkFun GPS-RTK2 Board - ZED-F9P (Qwiic)](https://www.sparkfun.com/products/15136)). 

Potential GPS module for future use: [SparkFun GPS-RTK Dead Reckoning Breakout - ZED-F9R (Qwiic)](https://www.sparkfun.com/products/16344), which has a 3D IMU sensor.

The driver was originally from [this link](https://github.com/KumarRobotics/ublox).

Modification has been made to meet our project purpose.

---

# Installation

Put the `src` folder under the catkin workspace directory, then use `catkin_make` when in the workspace directory. For development purpose (without `catkin_make install`), remember to use command `source <workspace_path>/devel/setup.bash` or `source <workspace_path>/devel/setup.zsh`.

---

# Launch

A sample launch file `ublox_device.launch` in the `launch` folder loads the parameters from a `zed_f9p.yaml` file in the same folder. Example command: `roslaunch ublox_device.launch`.

---

# Misc

- On Ubuntu VM, to run the launch file, `chmod` has to be used to change the access permission of the device file. Example command: `sudo chmod 777 /dev/ttyACM0`.

---

# Change history

2021/08/11 Tested on Ubuntu VM
