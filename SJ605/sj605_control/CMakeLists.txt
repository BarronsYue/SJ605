cmake_minimum_required(VERSION 2.8.3)
project(sj605_control)

find_package(catkin REQUIRED  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)
catkin_package()
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(sj605_instruction src/sj605_instruction.cpp)
target_link_libraries(sj605_instruction ${catkin_LIBRARIES})
