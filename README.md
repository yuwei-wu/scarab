scarab
=================
![Scarab](doc/SCARAB_3D_view.png "SCARAB")


ROS packages for MRSL scarab ground robots

[README for admins](doc/Admin.md)

[TODOS & Issues](doc/CurrentIssues.md)

```
tmux
roscore
roslaunch scarab rviz.launch agent:=scarab41
```

In a new terminal (ROS_MASTER_URI and ROS_IP are automatically set to the SSH_CLIENT)
```
tmux
ssh scarab41@192.168.131.41
roslaunch scarab scarab.launch
roslaunch scarab hfn.launch
```


Running Scarabs with Vicon

```
cd ~/ws_scarab/src
git clone https://github.com/KumarRobotics/motion_capture_system.git

roslaunch scarab vicon.launch
```
