#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import String
import dynamic_reconfigure.client
from controller_manager_msgs.srv import ListControllers

if __name__ == "__main__":
  rospy.init_node("start_gravity")

  try:
    rospy.wait_for_service("controller_manager/list_controllers")
    client = dynamic_reconfigure.client.Client("gazebo")

    while not rospy.is_shutdown():
      list_controllers = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)
      loaded_controllers = list_controllers()
      for controller in loaded_controllers.controller:
        if controller.name == "rh_trajectory_controller" and controller.state == "running":
          client.update_configuration({"gravity_z":-9.81})
          sys.exit(0)

  except rospy.ServiceException:
    rospy.logerr("Failed to set gravity")
