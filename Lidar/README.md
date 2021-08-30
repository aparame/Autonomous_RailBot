# Velodyne VLP-16
For technical specifications, please refer to the Datasheet at [this link](https://velodynelidar.com/downloads/#datasheets%20first)
The LIDAR will be used for object detection and autonomous driving alongside the Intel RealSense cameras
The LIDAR is also called as the Velodyne Puck

## Pre requisites
---
Before we start using this LIDAR, please note that the following steps have been tried and tested successfully on the below systems:
1) Ubuntu 16 and above on a Linux Machine
2) Jetson Nano and Xavier NX with Jetpack 4.5+

## Installation
---
The original installation tutorial is in [this link](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16).

* Check for the ethernet port name, type `ifconfig -a` in the terminal and identify the names of the ethernet port (normally `eth0`)
* The LIDAR's default static IP address is **192.168.1.201** (Please do not make changes to it).

---

# Launch
 Once the wrapper is installed, travel to `/catkin_ws`
* Open terminal, type and run `catkin_make` and then `source devel/setup.bash`
* If they are successful, then you can run a demo rospkg to visualize pointcloud in rviz by typing 
`roslaunch velodyne_pointcloud VLP16_points.launch`
* You would need to start rviz by typing
`rosrun rviz rviz -f velodyne`
**Make sure to save the rviz configuration in the /catkin_ws/src/velodyne folder for easier reconfig the next time**

---

# Change history

2021/08/30 Tested on Ubuntu 18.04 LTS

2021/08/18 Tested on Ubuntu 20.04 LTS


