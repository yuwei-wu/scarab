## Setup
First, setup workspace. The best way is to use Github token with HTTPS on scarab. First make sure you have access to the following repos, and then generate your personal github token.
```
mkdir -p ~/ws_{your project}/src
cd ~/ws_{your project}
catkin init
cd ~/ws_{your project}/src
git clone -b scarab_exp https://github.com/tyuezhan/scarab.git
git clone -b scarab https://github.com/tyuezhan/exploration_sem_pr.git
git clone -b factor_graph_yuezhan_scarab https://github.com/XuRobotics/generic-sloam.git
git clone -b sloam_real_robot https://github.com/tyuezhan/kr_mav_control.git
git clone -b scarab_sim https://github.com/tyuezhan/sem_detection.git
git clone https://github.com/tyuezhan/kr_f250_realsense.git
catkin build
```

Run `roscore` and `rviz` on your laptop. Your laptop is assumed to be ROS_MASTER
```
roscd exploration_manger/scripts
./tmux_scarab.sh
```

