# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 3.0.0)
project(tams_pr2_gripper_action)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp actionlib_msgs actionlib pr2_controllers_msgs)

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(tams_pr2_gripper_action src/pr2_gripper_action.cpp)
target_link_libraries(tams_pr2_gripper_action ${Boost_LIBRARIES} ${catkin_LIBRARIES})
set_target_properties(tams_pr2_gripper_action PROPERTIES OUTPUT_NAME pr2_gripper_action)

install(TARGETS tams_pr2_gripper_action
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
