# Individual installations

This is for development.

---
---

## Velodyne
For technical specifications, please refer to the Datasheet at [this link](https://velodynelidar.com/downloads/#datasheets%20first)
The LIDAR will be used for object detection and autonomous driving alongside the Intel RealSense cameras
The LIDAR is also called as the Velodyne Puck

### Pre requisites

Before we start using this LIDAR, please note that the following steps have been tried and tested successfully on the below systems:
1) Ubuntu 16 and above on a Linux Machine
2) Jetson Nano and Xavier NX with Jetpack 4.5+

### Installation

The original installation tutorial is in [this link](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16).

* Check for the ethernet port name, type `ifconfig -a` in the terminal and identify the names of the ethernet port (normally `eth0`)
* The LIDAR's default static IP address is **192.168.1.201** (Please do not make changes to it).

### Launch
 Once the wrapper is installed, travel to `/catkin_ws`
* Open terminal, type and run `catkin_make` and then `source devel/setup.bash`
* If they are successful, then you can run a demo rospkg to visualize pointcloud in rviz by typing
`roslaunch velodyne_pointcloud VLP16_points.launch`
* You would need to start rviz by typing
`rosrun rviz rviz -f velodyne`
**Make sure to save the rviz configuration in the /catkin_ws/src/velodyne folder for easier reconfig the next time**

### Change history

2021/08/30 Tested on Ubuntu 18.04 LTS

2021/08/18 Tested on Ubuntu 20.04 LTS

---

## Ublox

The ROS setup for GPS module (firmware version 9) ([SparkFun GPS-RTK2 Board - ZED-F9P (Qwiic)](https://www.sparkfun.com/products/15136)).

Potential GPS module for future use: [SparkFun GPS-RTK Dead Reckoning Breakout - ZED-F9R (Qwiic)](https://www.sparkfun.com/products/16344), which has a 3D IMU sensor.

The driver was originally from [this link](https://github.com/KumarRobotics/ublox).

Modification has been made to meet our project purpose.


### Installation

Put the `src` folder under the catkin workspace directory, then use `catkin_make` when in the workspace directory. For development purpose (without `catkin_make install`), remember to use command `source <workspace_path>/devel/setup.bash` or `source <workspace_path>/devel/setup.zsh`.


### Launch

A sample launch file `ublox_device.launch` in the `launch` folder loads the parameters from a `zed_f9p.yaml` file in the same folder. Example command: `roslaunch ublox_device.launch`.


### Misc

- On Ubuntu VM, to run the launch file, `chmod` has to be used to change the access permission of the device file. Example command: `sudo chmod 777 /dev/ttyACM0`.


### Change history

2021/08/11 Tested on Ubuntu VM
