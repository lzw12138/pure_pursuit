## ROS

### Dependencies

You can install them by running:

```html
sudo apt-get install ros-melodic-tf2-geometry-msgs ros-melodic-ackermann-msgs ros-melodic-joy ros-melodic-map-server
```

### Installation

```
cd ~/catkin_ws/src
git clone https://github.com/lzw12138/pure_pursuit.git
cd .. && catkin_make
source devel/setup.bash
```

### Quick Start

```
roslaunch f1tenth_simulator simulator.launch
rosrun tianracer_navigation Pure_Pursuit
```

