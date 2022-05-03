#! /usr/bin/env python

from pr2_mechanism_msgs.srv import *
from std_srvs.srv import *
from std_msgs.msg import Bool
import rospy


global run
run = False

global pr2_controllers
global loose_controllers

def toggle_service(req):
    global run
    run = req.data

    # switch to controllers with low gains / normal controllers
    rospy.wait_for_service('pr2_controller_manager/switch_controller')
    try:
        switch_controllers = rospy.ServiceProxy('pr2_controller_manager/switch_controller', SwitchController)
        if run:
            resp1 = switch_controllers(loose_controllers, pr2_controllers, SwitchControllerRequest.STRICT)
        else:
            resp1 = switch_controllers(pr2_controllers, loose_controllers, SwitchControllerRequest.STRICT)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return

    if not resp1:
        print "switching controllers failed"
        return

    # inform our custom mannequin mode to start/stop adjusting the target state on joint state errors
    for i in loose_controllers:
        service_id = i + '/set_trajectory_lock'
        rospy.wait_for_service(service_id)
        try:
            toggle_lock = rospy.ServiceProxy(service_id, SetBool)
            resp2 = toggle_lock(run)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    state_publisher.publish(run)
    if run:
        message = 'mannequin mode active'
    else:
        message = 'mannequin mode inactive'
    return SetBoolResponse(True, message)

if __name__ == "__main__":
    rospy.init_node("mannequin_mode_server")

    pr2_controllers = rospy.get_param('~pr2_controllers')   #['head_traj_controller', 'l_arm_controller', 'r_arm_controller']
    loose_controllers = rospy.get_param('~loose_controllers')   #['head_traj_controller_loose', 'l_arm_controller_loose', 'r_arm_controller_loose']

    state_publisher = rospy.Publisher('mannequin_mode_active', Bool, latch= True)
    state_publisher.publish(False)

    s = rospy.Service('set_mannequin_mode', SetBool, toggle_service)

    rospy.spin()
