#!/usr/bin/env python

# by Michael Goerner @ 20161011

# This script publishes 0 joint states for all joints of the right hand.
# This is useful when the hand is not initialized with the whole setup

import rospy
import sensor_msgs.msg

if __name__ == '__main__':
  rospy.init_node('joint_state_publisher_hand')

  pub= rospy.Publisher('joint_states', sensor_msgs.msg.JointState, queue_size= 1)

  joints= ['rh_WRJ2', 'rh_WRJ1', 'rh_FFJ4', 'rh_FFJ3', 'rh_FFJ2', 'rh_FFJ1', 'rh_LFJ5', 'rh_LFJ4', 'rh_LFJ3', 'rh_LFJ2', 'rh_LFJ1', 'rh_MFJ4', 'rh_MFJ3', 'rh_MFJ2', 'rh_MFJ1', 'rh_RFJ4', 'rh_RFJ3', 'rh_RFJ2', 'rh_RFJ1', 'rh_THJ5', 'rh_THJ4', 'rh_THJ3', 'rh_THJ2', 'rh_THJ1']
  js= sensor_msgs.msg.JointState(
    name= joints,
    position= [0.0]*len(joints),
    velocity= [0.0]*len(joints),
    effort= [0.0]*len(joints))


  r= rospy.Rate(60)
  while not(rospy.is_shutdown()):
    js.header.stamp= rospy.Time.now()
    pub.publish(js)
    r.sleep()
