# Intel RealSense D455 Camera
For technical specifications, please refer to the Datasheet at [this link](https://www.intelrealsense.com/wp-content/uploads/2020/06/Intel-RealSense-D400-Series-Datasheet-June-2020.pdf)
The camera will be used as the RGB raw image data input

## Pre requisites
---
Before we start using this camera, please note that the following steps have been tried and tested successfully on the below systems:
1) Ubuntu 16 and above on a Linux Machine
2) Jetson Nano and Xavier NX with Jetpack 4.5+

## Installation
---
### Step 1 : Install the librealsense SDK 2.0
The **Intel RealSense SDK 2.0** is a cross-platform library for Intel RealSense Cameras.
Once installed, you will be capable of viewing the camera using a built in GUI on Windows/Linux/MAC machines. It is basically providing you with libraries as well as acting as an front end application for your camera

To install the **SDK** on **Linux** machines (*inlcuding Jetson*), follow the steps provided [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)

* To test if you have successfully installed the SDK, simply connect your camera and type `realsense-viewer` in the terminal
    
---
### Step 2 : Install the ROS wrapper for Intel RealSense Devices 

Once you have installed the **librealsense** SDK on the device and tested that it's working, the **ROS** wrapper for the same can be installed as well.

To install the wrapper, follow the steps provided [here](https://github.com/IntelRealSense/realsense-ros)

*Please note that if the above steps have been fulfilled, then you can skip to* '**Step 2: Install Intel RealSense ROM from Sources**' *directly in the above* [link](https://github.com/IntelRealSense/realsense-ros)

# Launch
 Once the wrapper is installed, travel to `/catkin_ws`
* Open terminal, type and run `catkin_make` and then `source devel/setup.bash`
* If they are successful, then you can run a demo rospkg to visualize pointcloud in rviz by typing 
`roslaunch realsense2_camera demo_pointcloud.launch`

---

# Change history

2021/08/11 Tested on Ubuntu 20.04 LTS

