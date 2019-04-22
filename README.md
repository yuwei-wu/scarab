scarab
=================
![Scarab](doc/SCARAB_3D_view.png "SCARAB")


ROS packages for MRSL scarab ground robots

[README for admins](doc/Admin.md)

[TODOS & Issues](doc/CurrentIssues.md)

On your computer/laptop
```
tmux
roscore
roslaunch scarab rviz.launch agent:=scarab41
```

In a new terminal (ROS_MASTER_URI and ROS_IP are automatically set to the SSH_CLIENT)
```
tmux
ssh scarab@192.168.131.41
roslaunch scarab navigation.launch
```

Running Scarabs with Vicon on your computer/laptop create a workspace with following packages and compile. 

```
cd ~/ws_scarab/src
git clone https://github.com/KumarRobotics/motion_capture_system.git
git clone https://github.com/KumarRobotics/scarab

tmux
export ROS_IP=$(ifconfig | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p')
roslaunch scarab vicon.launch

```

On the scarab
```
tmux
ssh scarab@192.168.131.41
roslaunch scarab navigation.launch use_vicon:=true
```

Publish a geometry_msgs/PoseStamped on /scarab41/move_base_simple/goal
If the robot does not move, your ROS_IP is not set. Use following

```
export ROS_IP=$(ifconfig | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p')
```
