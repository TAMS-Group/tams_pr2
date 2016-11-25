#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

# Author: Wim Meeussen
# Calibrates the PR-2 in a safe sequence


import roslib
roslib.load_manifest('pr2_bringup')
import rospy
import getopt
import yaml
import sys
from std_msgs.msg import *
from pr2_mechanism_msgs.srv import LoadController, UnloadController, SwitchController, SwitchControllerRequest, ListControllers
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from pr2_controllers_msgs.srv import QueryCalibrationState
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from sensor_msgs.msg import *



calibration_params_namespace = "calibration_controllers"
load_controller = rospy.ServiceProxy('pr2_controller_manager/load_controller', LoadController)
unload_controller = rospy.ServiceProxy('pr2_controller_manager/unload_controller', UnloadController)
switch_controller = rospy.ServiceProxy('pr2_controller_manager/switch_controller', SwitchController)
list_controllers = rospy.ServiceProxy('pr2_controller_manager/list_controllers', ListControllers)

hold_position = {'r_shoulder_pan': -0.7, 'r_elbow_flex': -2.0, 'r_upper_arm_roll': 0.0, 'r_shoulder_lift': 1.0}

def get_controller_name(joint_name):
    return calibration_params_namespace+"/calibrate/cal_%s" % joint_name

def get_holding_name(joint_name):
    return "%s/hold/%s_position_controller" % (calibration_params_namespace, joint_name)

def get_service_name(joint_name):
    return '%s/is_calibrated'%get_controller_name(joint_name)

global last_joint_states
last_joint_states = None
def joint_states_cb(msg):
    global last_joint_states
    last_joint_states = msg
rospy.Subscriber('joint_states', JointState, joint_states_cb)
    
global motors_halted
motors_halted = None
def motor_state_cb(msg):
    global motors_halted
    motors_halted = msg.data
    rospy.logdebug("motors halted = %d"%motors_halted)
rospy.Subscriber('pr2_ethercat/motors_halted', Bool, motor_state_cb)

pub_diag = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10) 
def diagnostics(level, msg_short, msg_long):
    if level == 0:
        rospy.loginfo(msg_long)        
    elif level == 1:
        rospy.logwarn(msg_long)
    elif level == 2:
        rospy.logerr(msg_long)
    d = DiagnosticArray() 
    d.header.stamp = rospy.Time.now() 
    ds = DiagnosticStatus() 
    ds.level = level
    ds.message = msg_long
    ds.name = msg_short
    d.status = [ds] 
    pub_diag.publish(d) 



class HoldingController:
    def __init__(self, joint_name):
        self.joint_name = joint_name
        rospy.logdebug("Loading holding controller: %s" %get_holding_name(joint_name))
        resp = load_controller(get_holding_name(joint_name)) 
        if resp.ok:
            rospy.logdebug("Starting holding controller for joint %s."%joint_name)
            switch_controller([get_holding_name(joint_name)], [], SwitchControllerRequest.STRICT)
            self.pub_cmd = rospy.Publisher("%s/command" %get_holding_name(joint_name), Float64, latch=True, queue_size=10)
        else:
            rospy.logerr("Failed to load holding controller for joint %s."%joint_name)
            raise Exception('Failure to load holding controller')
                                  
    def __del__(self):
        switch_controller([], [get_holding_name(self.joint_name)], SwitchControllerRequest.STRICT)
        unload_controller(get_holding_name(self.joint_name))
        self.pub_cmd.unregister()
                               
    def hold(self, position):
        self.pub_cmd.publish(Float64(position))


class Calibrate:
    def __init__(self, joints):
        self.joints = []
        self.hold_controllers = []
        self.services = {}

        # spawn calibration controllers for all joints
        for j in joints:
            rospy.logdebug("Loading controller: %s" %get_controller_name(joint))
            resp = load_controller(get_controller_name(j)) 
            if resp.ok:
                # get service call to calibration controller to check calibration state
                rospy.logdebug("Waiting for service: %s" %get_service_name(j))
                rospy.wait_for_service(get_service_name(j))
                self.services[j] = rospy.ServiceProxy(get_service_name(j), QueryCalibrationState)
                self.joints.append(j)
            else:
                rospy.logerr("Failed to load calibration for joint %s. Skipping this joint"%j)


    def __del__(self):
        # stop controllers that were started
        switch_controller([], [get_controller_name(j) for j in self.joints], SwitchControllerRequest.BEST_EFFORT)        

        # kill controllers that were loaded
        for j in self.joints:
            unload_controller(get_controller_name(j))

    def is_calibrated(self):
        # check if joints are calibrated
        for j in self.joints:
            if self.services[j]().is_calibrated:
                rospy.logdebug("joint %s is already calibrated"%j)
            else:
                rospy.logdebug("joint %s needs to be calibrated"%j)
                return False
        return True


    def calibrate(self):
        # start all calibration controllers
        rospy.logdebug("Start calibrating joints %s"%self.joints)
        switch_controller([get_controller_name(j) for j in self.joints], [], SwitchControllerRequest.STRICT)

        # wait for joints to calibrate
        start_time = rospy.Time.now()
        while not self.is_calibrated():
            if motors_halted and motors_halted == 1:
                diagnostics(2, 'Calibration on hold', 'Calibration is on hold because motors are halted. Enable the run-stop')
                start_time = rospy.Time.now()
                rospy.sleep(1.0)
            elif rospy.Time.now() > start_time + rospy.Duration(30.0):  # time for spine to go up is 29 seconds
                diagnostics(2, 'Calibration stuck', 'Joint %s is taking a long time to calibrate. It might be stuck and need some human help'%self.joints)
                rospy.sleep(1.0)                    
            rospy.sleep(0.1)

        rospy.logdebug("Stop calibration controllers for joints %s"%self.joints)
        switch_controller([], [get_controller_name(j) for j in self.joints], SwitchControllerRequest.BEST_EFFORT)

        # hold joints in place
        rospy.logdebug("Loading holding controllers for joints %s"%self.joints)
        self.hold_controllers = []
        for j in self.joints:
            if j in hold_position:
                holder = HoldingController(j)
                holder.hold(hold_position[j])
                self.hold_controllers.append(holder)



            
def main():
    try:
        rospy.init_node('calibration', anonymous=True, disable_signals=True)
        calibration_start_time = rospy.Time.now()

        rospy.wait_for_service('pr2_controller_manager/load_controller')
        rospy.wait_for_service('pr2_controller_manager/switch_controller')
        rospy.wait_for_service('pr2_controller_manager/unload_controller')

        
        # parse options
        args = rospy.myargv()

        # load controller configuration
        rospy.loginfo("Loading controller configuration on parameter server...")
        pr2_controller_configuration_dir = roslib.packages.get_pkg_dir('tams_pr2_controller_configuration')
        calibration_yaml = '%s/pr2_calibration_controllers.yaml' % pr2_controller_configuration_dir
        hold_yaml = '%s/pr2_joint_position_controllers.yaml' % pr2_controller_configuration_dir
        if len(args) < 3:
            rospy.loginfo("No yaml files specified for calibration and holding controllers, using defaults")
        else:
            calibration_yaml = args[1]
            hold_yaml  = args[2]
        rospy.set_param(calibration_params_namespace+"/calibrate", yaml.load(open(calibration_yaml)))
        rospy.set_param(calibration_params_namespace+"/hold", yaml.load(open(hold_yaml)))

        joints_status = False

        # define calibration sequence objects
        arm_list = [['r_shoulder_pan'], ['r_shoulder_lift'], ['r_upper_arm_roll'], ['r_elbow_flex'], ['r_forearm_roll']]

        torso = Calibrate([['torso_lift']])
        if not torso.is_calibrated():
            print "Torso is not calibrated"
            sys.exit(1)
        
        torso_holder = HoldingController('torso_lift')
        torso_holder.hold(0.25)
        rospy.sleep(5.0)
        rospy.loginfo('Moving up spine to allow arm to calibrate')
        
        for joint in arm_list:
            raw_input("Press 'Enter' to calibrate joint %s"%joint)
            calibrate_joint = Calibrate(joint)
            calibrate_joint.calibrate()

        rospy.loginfo('Moving down spine after arm calibration')
        torso_holder.hold(0.01)
        rospy.sleep(20.0)

        joints_status = True

    except Exception as e:
        rospy.logerr("Calibration failed: %s" % str(e))
        
    finally:
        rospy.loginfo("Bringing down calibration node")

        rospy.set_param(calibration_params_namespace, "")
        del torso_holder

        if not joints_status:
            rospy.logerr("Mechanism calibration failed")
        else:
            rospy.loginfo('Calibration completed in %f sec' %(rospy.Time.now() - calibration_start_time).to_sec())
            rospy.spin()
            

if __name__ == '__main__': main()
