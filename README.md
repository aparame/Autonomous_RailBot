# RailBot ROS software repository

## Installation

1. Put the `src` folder under the catkin workspace directory, then use `catkin_make` when in the workspace directory. 

2. Install `rosdep` package if it is not installed by `sudo apt-get install python-rosdep`.

3. Inside your workspace, install and update all dependencies by using `rosdep install --from-paths src --ignore-src --rosdistro melodic -y`.

4. For development purpose (without `catkin_make install`), remember to use command `source <workspace_path>/devel/setup.bash` or `source <workspace_path>/devel/setup.zsh` depending on your shell.

---

## Launch


---
