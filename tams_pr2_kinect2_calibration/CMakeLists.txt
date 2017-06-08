cmake_minimum_required(VERSION 2.8.3)
project(tams_pr2_kinect2_calibration)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  moveit_ros_planning_interface
  moveit_msgs
  visualization_msgs
  geometry_msgs
  actionlib
  pr2_controllers_msgs
)

add_service_files(
  FILES Kinect2CalibrationControl.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(calibration_move src/calibration_move.cpp)
add_dependencies(calibration_move ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(calibration_move ${catkin_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS calibration_move DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
