# RailBot ROS software repository

## Installation

Install `rosdep` package if it is not installed by `sudo apt-get install python-rosdep`.

Inside your workspace, install and update all dependencies by using `rosdep install --from-paths src --ignore-src --rosdistro melodic -y`.

Put the `src` folder under the catkin workspace directory, then use `catkin_make` when in the workspace directory. For development purpose (without `catkin_make install`), remember to use command `source <workspace_path>/devel/setup.bash` or `source <workspace_path>/devel/setup.zsh` depending on your shell.

---

## Launch


---
