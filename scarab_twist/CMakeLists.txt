cmake_minimum_required(VERSION 2.8.3)
project(scarab_twist)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Curses)

catkin_package(CATKIN_DEPENDS roscpp DEPENDS Curses)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(joy_to_twist src/joy_to_twist.cpp)
target_link_libraries(joy_to_twist ${catkin_LIBRARIES})

add_executable(key_to_twist src/key_to_twist.cpp)
target_link_libraries(key_to_twist ${catkin_LIBRARIES} ${CURSES_LIBRARIES})

install(TARGETS
    joy_to_twist
    key_to_twist
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)