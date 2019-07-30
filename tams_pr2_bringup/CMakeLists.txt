cmake_minimum_required(VERSION 2.8.3)
project(tams_pr2_bringup)
find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(
  PROGRAMS
    scripts/calibrate_pr2.py
    scripts/calibrate_right_arm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
