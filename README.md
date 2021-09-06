# RailBot ROS software repository

## Prerequisite

1. **Default Python version**: Since ROS melodic is used, the default python version should be python2, which means `/usr/bin/python` should point to `/usr/bin/python2` (or self installed python2). If not, please use `sudo update-alternatives --install /usr/bin/python python /usr/bin/python2 1` and `sudo update-alternatives --config python` to configure it.

2. **Intel® RealSense™ SDK 2.0**: a cross-platform library for Intel® RealSense™ depth cameras. More details are at [this link](https://github.com/IntelRealSense/librealsense).
2.1 sdfaas
2.2 sdafds

---

## Installation

1. Put the `sensor` folder under the `src` in the catkin workspace directory

2. Install `rosdep` package if it is not installed by `sudo apt-get install python-rosdep`. Inside your workspace, install and update all dependencies by using `rosdep install --from-paths src --ignore-src --rosdistro melodic -y`

3. Use `catkin_make` when in the workspace directory

4. For development purpose (without `catkin_make install`), remember to use command `source <workspace_path>/devel/setup.bash` or `source <workspace_path>/devel/setup.zsh` depending on your shell

---

## I/O ports for sensor devices

### GPS (USB)

Default value: `/dev/ttyACM0`. Check in `/dev/` folder in the OS. Need to adjust it in `sensor/ublox/ublox_gps/config/zed_f9p.yaml` file (according to the default launch file).

### Lidar (Ethernet)

Default value: `eth0`. Check for the ethernet port name, type `ifconfig -a` or `ip a` in the terminal and identify the names of the ethernet port (normally `eth0`).

After connecting the ethernet cable to the Lidar, type in `nmcli dev disconnect eth0` and `sudo ifconfig eth0 192.168.1.x netmask 255.255.255.0`. x is a number between 1 to 254, except 201, since the LIDAR's default static IP address is **192.168.1.201** (Do not change it). 

To revert back to internet connection, type in `nmcli dev disconnect eth0` and `sudo dhclient -v eth0` with cable connected.

---

## Launch

**Simply use `roslaunch sensor.launch`!**

For development and modification purpose, the individual launch files can be modified. 

GPS: `sensor/ublox/ublox_gps/launch/ublox_device.launch`

Lidar: `sensor/velodyn/velodyne_pointcloud/launch/VLP16_points.launch`

---
