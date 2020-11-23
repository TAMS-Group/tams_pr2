cmake_minimum_required(VERSION 2.8.3)
project(tams_pr2_calibration)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_ros_planning_interface
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(left_arm_sample_poses src/left_arm_sample_poses.cpp)
target_link_libraries(left_arm_sample_poses ${catkin_LIBRARIES})

add_executable(right_arm_sample_poses src/right_arm_sample_poses.cpp)
target_link_libraries(right_arm_sample_poses ${catkin_LIBRARIES})

add_executable(l_forearm_sample_poses src/l_forearm_sample_poses.cpp)
add_dependencies(l_forearm_sample_poses ${catkin_EXPORTED_TARGETS})
target_link_libraries(l_forearm_sample_poses ${catkin_LIBRARIES})

