# lunarteam-soft
## Overview
This is a repository created as a temporary repository for PUTLunarTeam software team. All is bound to change.

All packages are created and tested on ROS2 Humble

### Building

To build from source, clone the latest version from this repository into your colcon workspace and build the package using:
~~~
  cd <your_workspace_name>/src
  git clone https://github.com/ReQ1600/lunarteam-soft.git
  cd ..
  rosdep install --from-paths . --ignore-src
  colcon build
~~~

### Docker
To be added
