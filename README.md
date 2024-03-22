# kinova_gen3_control_cpp
### Overview

PUTLunarTeam repository for ROS2 package controlling kinova gen3 robotic arm

Package created and tested on ROS2 Humble

Project uses kinova Gen 3 robotic arm

### Dependencies

- ros2_kortex
- moveit2
- moveit_resources

### Installing

Install ros2_kortex from Kinovarobotics on github (https://github.com/Kinovarobotics/ros2_kortex) do what readme tells you to do.

Clone moveit_resources from https://github.com/ros-planning/moveit_resources.git

Source your workspace and download moveit2:
~~~
  git clone https://github.com/ros-planning/moveit2.git -b $ROS_DISTRO
  for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
  rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
~~~

### Building

To build from source, clone the latest version from this repository into your colcon workspace and build the package using:
~~~
  cd <your_workspace_name>/src
  git clone https://github.com/ReQ1600/kinova_gen3_control_cpp.git
  cd ..
  rosdep install --from-paths . --ignore-src
  colcon build
~~~

### Running

Run the simulation with:
~~~
  ros2 launch kortex_bringup gen3.launch.py \
  robot_ip:=yyy.yyy.yyy.yyy \
  use_fake_hardware:=true
~~~
Then run the node with ros2 run (will later be changed to a common launch file)
