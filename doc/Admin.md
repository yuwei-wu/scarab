Scarab setup instructions for Ubuntu 16.04 and ROS Kinetic

Enable grapics support for kernels 4.13.0-41-generic and below
$ sudo update-pciids
$ sudo vim /etc/default/grub

Add i915.alpha_support=1 to GRUB_CMDLINE_LINUX_DEFAULT="" in /etc/default/grub

$ sudo update-grub

Install wifi drivers - if not supported by default
$ cd ~/Downloads
$ wget https://github.com/lwfinger/rtlwifi_new/archive/extended.zip
$ unzip extended.zip
$ cd rtlwifi_new-extended
$ make
$ sudo make install
$ sudo modprobe rtl8822be

Scarab system config files are located in
https://github.com/KumarRobotics/scarab/tree/master/config

Copy network interfaces and wpa_supplicant file
$ sudo cp interface /etc/network/
$ sudo cp wpa_supplicant.conf /etc/wpa_supplicant/

Copy udev rules for setting USB persissions
$ sudo cp 99-scarab.rules /etc/udev/rules.d

Install ROS Kinetic (ros-kinetic-desktop-full)
http://wiki.ros.org/kinetic/Installation

Other ROS packages
$ sudo apt-get install ros-kinetic-urg-node

Dependencies to install
$ sudo apt-get install libarmadillo-dev libncurses5-dev libnl-3-dev libnl-route-3-dev libnl-genl-3-dev libnl-nf-3-dev libcgal-dev libcgal-qt5-dev

Install other packages
$ sudo apt-get install openssh-server emacs vim tmux tmuxinator

$ cd ~/
$ mkdir ws_scarab

