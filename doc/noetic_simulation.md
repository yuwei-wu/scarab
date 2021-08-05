# Scarab simulator setup instructions for Ubuntu 20.04 and ROS Noetic
Required: Ubuntu 20.04 with ROS noetic installed

Install dependency:
```
sudo apt-get install libcgal-dev libnl-3-dev libnl-route-3-dev libnl-genl-3-dev libnl-nf-3-dev
```

Go to your catkin workspace, inside ```catkin_ws```:
```
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release
cd src
git clone -b devel_noetic git@github.com:tyuezhan/scarab.git
```

Build and run:
```
catkin build
roslaunch scarab scarab_sim.launch
```

If you want to control scarab with keyboad:
```
roslaunch scarab_twist sim_joystick.launch
```

Send waypoints in rviz:

In rviz, use the ```2D Nav Goal``` tool, which publish waypoints to: [agent name]/move_base_simple/goal