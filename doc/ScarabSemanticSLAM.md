## Setup
**This is using scarab**
First, setup workspace. The best way is to use Github token with HTTPS on scarab. First make sure you have access to the following repos, and then generate your personal github token.
```
mkdir -p ~/ws_{your project}/src
cd ~/ws_{your project}
catkin init
cd ~/ws_{your project}/src
git clone https://github.com/catkin/catkin_simple.git 
git clone -b scarab_exp https://github.com/tyuezhan/scarab.git
git clone -b scarab https://github.com/tyuezhan/exploration_sem_pr.git
git clone -b factor_graph_yuezhan_scarab https://github.com/XuRobotics/generic-sloam.git
git clone -b sloam_real_robot https://github.com/tyuezhan/kr_mav_control.git
git clone -b scarab_sim https://github.com/tyuezhan/sem_detection.git
git clone https://github.com/tyuezhan/kr_f250_realsense.git
```

Install realsense driver (this require scarab_admin account):
```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key  F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key  F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
sudo apt-get install ros-noetic-ddynamic-reconfigure
```


Test realsense: realsense-viewer Check if the top left corner says USB 3.x

Compile workspace
```
catkin build
```

Single robot experiment, directly run everything on scarab:
```
roscd exploration_manger/scripts
./tmux_scarab.sh
```

