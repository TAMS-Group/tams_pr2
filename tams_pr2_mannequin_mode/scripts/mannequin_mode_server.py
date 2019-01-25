#! /usr/bin/env python

from pr2_mechanism_msgs.srv import *
from std_srvs.srv import *
import rospy

rospy.init_node("mannequin_mode_server")
global run
run = False
pr2_controllers = rospy.get_param('~pr2_controllers')   #['head_traj_controller', 'l_arm_controller', 'r_arm_controller']
loose_controllers = rospy.get_param('~loose_controllers')   #['head_traj_controller_loose', 'l_arm_controller_loose', 'r_arm_controller_loose']

def toggle_service(req):
    global run
    run = req.data
    rospy.wait_for_service('pr2_controller_manager/switch_controller')
    try:
        switch_controllers = rospy.ServiceProxy('pr2_controller_manager/switch_controller', SwitchController)
        if run:
            resp1 = switch_controllers(loose_controllers, pr2_controllers, SwitchControllerRequest.STRICT)
        else:
            resp1 = switch_controllers(pr2_controllers, loose_controllers, SwitchControllerRequest.STRICT)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        run = not run
        return

    if not resp1:
        print "switching controllers failed"
        run = not run
        return
    
    for i in loose_controllers:
        service_id = i + '/set_trajectory_lock'
        rospy.wait_for_service(service_id)
        try:
            toggle_lock = rospy.ServiceProxy(service_id, SetBool)
            resp2 = toggle_lock(run)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    if run:
        message = 'mannequin mode active'
    else:
        message = 'mannequin mode inactive'
    return SetBoolResponse(True, message)

s = rospy.Service('set_mannequin_mode', SetBool, toggle_service)
rospy.spin()
