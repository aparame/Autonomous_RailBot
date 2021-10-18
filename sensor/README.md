# Individual Installations

This is for development.

---
---

## Velodyne
For technical specifications, please refer to the Datasheet at [this link](https://velodynelidar.com/downloads/#datasheets%20first)
The LIDAR will be used for object detection and autonomous driving alongside the Intel RealSense cameras
The LIDAR is also called as the [Velodyne Puck](https://velodynelidar.com/products/puck/).

### Source

The original driver is in [this link](http://wiki.ros.org/velodyne).

### Visualization
* You would need to start rviz by typing
`rosrun rviz rviz -f velodyne`


---

## Ublox

The ROS setup for GPS module (firmware version 9) ([SparkFun GPS-RTK2 Board - ZED-F9P (Qwiic)](https://www.sparkfun.com/products/15136)).

Potential GPS module for future use: [SparkFun GPS-RTK Dead Reckoning Breakout - ZED-F9R (Qwiic)](https://www.sparkfun.com/products/16344), which has a 3D IMU sensor.

Modification has been made to meet our project purpose.

### Source

The driver was originally from [this link](https://github.com/KumarRobotics/ublox).

---

## Intel Realsense Depth Camera

The Intel Realsense Depth Camera is [model D455](https://www.intelrealsense.com/depth-camera-d455/). 

### Source

The library for this camera is in [this link](http://wiki.ros.org/RealSense).
The driver was originally from [this link](http://wiki.ros.org/realsense_camera).

---

## Razor IMU

The Razor inertial measurement unit (IMU) is of 9 degree freedom

### Source

The driver was originally from [this link](http://wiki.ros.org/razor_imu_9dof).

