cmake_minimum_required(VERSION 2.8.3)
project(scarab_mpc)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  scarab_msgs
  std_msgs
  visualization_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/mpc_controller_node.py
  scripts/mpc_multi_sim_test_monitor.py
  scripts/mpc_sim_test_monitor.py
  scripts/tf_pose_publisher_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
