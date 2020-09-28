cmake_minimum_required(VERSION 2.8.3)
project(tams_pr2_bringup)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
        roscpp
        moveit_core
        moveit_ros_planning_interface
        moveit_msgs
        geometry_msgs
        std_msgs
        moveit_visual_tools
        tf2_geometry_msgs
        )

catkin_package(   CATKIN_DEPENDS
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(
  PROGRAMS
    scripts/calibrate_pr2.py
    scripts/calibrate_right_arm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(pr2_demo_test src/pr2_demo_test.cpp)
add_dependencies(pr2_demo_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(pr2_demo_test ${catkin_LIBRARIES})