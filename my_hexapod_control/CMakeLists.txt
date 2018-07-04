cmake_minimum_required(VERSION 2.8.3)
project(my_hexapod_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_hexapod_control
#  CATKIN_DEPENDS roscpp rospy std_msgs tf
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)




add_library (gait SHARED src/gait.cpp)
add_library (ik SHARED src/ik.cpp)
add_library (control SHARED src/control.cpp)

add_executable( my_hexapod_controller src/my_hexapod_controller.cpp)
target_link_libraries( my_hexapod_controller ${catkin_LIBRARIES} gait ik control)

