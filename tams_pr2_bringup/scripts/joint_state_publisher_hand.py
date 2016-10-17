#!/usr/bin/env python

# by Michael Goerner @ 20161011

# This script publishes 0 joint states for all joints of the right hand.
# This is useful when the hand is not initialized with the whole setup

import rospy
import sensor_msgs.msg

if __name__ == '__main__':
  rospy.init_node('joint_state_publisher_hand')

  pub= rospy.Publisher('joint_states', sensor_msgs.msg.JointState, queue_size= 1)

  joints= ['r_hand_WRJ2', 'r_hand_WRJ1', 'r_hand_FFJ4', 'r_hand_FFJ3', 'r_hand_FFJ2', 'r_hand_FFJ1', 'r_hand_LFJ5', 'r_hand_LFJ4', 'r_hand_LFJ3', 'r_hand_LFJ2', 'r_hand_LFJ1', 'r_hand_MFJ4', 'r_hand_MFJ3', 'r_hand_MFJ2', 'r_hand_MFJ1', 'r_hand_RFJ4', 'r_hand_RFJ3', 'r_hand_RFJ2', 'r_hand_RFJ1', 'r_hand_THJ5', 'r_hand_THJ4', 'r_hand_THJ3', 'r_hand_THJ2', 'r_hand_THJ1']
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
