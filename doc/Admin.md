# Scarab setup instructions for Ubuntu 16.04 and ROS Kinetic

Install Ubunutu 16.04. Make admin user `scarab_admin`

Enable graphics support for kernels 4.13.0-41-generic and below
```
sudo update-pciids
sudo edit /etc/default/grub
```

Add i915.alpha_support=1 to GRUB_CMDLINE_LINUX_DEFAULT="quiet splash" in /etc/default/grub
```
sudo update-grub
sudo reboot
```

Install wifi drivers - if not supported by default (Note, this has to be installed again if the kernel is updated)
```
cd ~/Downloads
wget https://github.com/lwfinger/rtlwifi_new/archive/extended.zip
unzip extended.zip
cd rtlwifi_new-extended
make
sudo make install
sudo modprobe rtl8822be
```

Scarab system config files are located in
https://github.com/KumarRobotics/scarab/tree/devel_cleanup/config



Copy network interfaces and wpa_supplicant file
```
cd ~/Downloads
wget https://raw.githubusercontent.com/KumarRobotics/scarab/devel_cleanup/config/wpa_supplicant.conf
wget https://raw.githubusercontent.com/KumarRobotics/scarab/devel_cleanup/config/interfaces
```
Change the ip in interfaces file to reflect scarab id (example scarab44 will be 192.168.131.44, replace all ips)
```
sudo cp interfaces /etc/network/
sudo cp wpa_supplicant.conf /etc/wpa_supplicant/
```

Copy udev rules for setting USB persissions
```
https://raw.githubusercontent.com/KumarRobotics/scarab/devel_cleanup/config/99-scarab.rules
sudo cp 99-scarab.rules /etc/udev/rules.d
sudo udevadm control --reload-rules && udevadm trigger
cd ..
rm -rf config
```

Install ROS Kinetic (ros-kinetic-desktop-full)
http://wiki.ros.org/kinetic/Installation

Other ROS packages
```
sudo apt-get install ros-kinetic-urg-node
```

Scarab dependencies to install
```
sudo apt-get install libarmadillo-dev libncurses5-dev libnl-3-dev libnl-route-3-dev libnl-genl-3-dev libnl-nf-3-dev libcgal-dev libcgal-qt5-dev
```

Install other packages
```
sudo apt-get install openssh-server vim emacs tmux tmuxinator python-catkin-tools git
```

Setup catkin workspace
```
cd ~/
mkdir ws_scarab
cd ws_scarab
mkdir src
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release
cd src
git clone https://github.com/KumarRobotics/scarab.git
cd scarab
git checkout -t origin/devel_cleanup
catkin build
```

Change the AGENT to reflect the current scarab id
```
echo "export AGENT=scarab44" >> ~/.bashrc
source ~/.bashrc
source ~/ws_scarab/devel/setup.bash

Turn on the H-BRDG and ROBO CLAW PWR switches

roslaunch scarab scarab.launch
```

