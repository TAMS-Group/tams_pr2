#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState


joints_of_interest_head = [
     'head_pan_joint',
     'head_tilt_joint'
     ]
joints_of_interest_right_arm = [
     'r_shoulder_pan_joint',
     'r_shoulder_lift_joint',
     'r_upper_arm_roll_joint',
     'r_elbow_flex_joint',
     'r_forearm_roll_joint'
     ]

joints_of_interest_left_arm = [
     'l_shoulder_pan_joint',
     'l_shoulder_lift_joint',
     'l_upper_arm_roll_joint',
     'l_elbow_flex_joint',
     'l_forearm_roll_joint',
     'l_wrist_flex_joint',
     'l_wrist_roll_joint'
     ]


def listener():
    rospy.init_node('getPose', anonymous=True)

    # rospy.Subscriber("/joint_states", JointState, callback2, queue_size=1)
    sample_count =0
    while not rospy.is_shutdown():
       # use wait_for_message instead of subscriber since we need to wait for input inside of the callback
       data = rospy.wait_for_message("/joint_states", JointState)

       # print(data.header.seq)
       head = []
       r_arm = []
       l_arm = []
       for j in range(len(joints_of_interest_head)):
         for i in range(len(data.name)):
           if data.name[i] == joints_of_interest_head[j]:
             head.append(data.position[i])
             print(data.name[i] + ': ' + str(data.position[i]))

       for j in range(len(joints_of_interest_right_arm)):
         for i in range(len(data.name)):
           if data.name[i] == joints_of_interest_right_arm[j]:
             r_arm.append(data.position[i])
             print(data.name[i] + ': ' + str(data.position[i]))

       for j in range(len(joints_of_interest_left_arm)):
         for i in range(len(data.name)):
           if data.name[i] == joints_of_interest_left_arm[j]:
             print(data.name[i] + ': ' + str(data.position[i]))
             l_arm.append(data.position[i])
       print '--------------------'
       print('Sample number: ' + str(sample_count))
       print('Head: ')
       print('{' + str(head)[1:-1] + '},')
       print('Right arm: ')
       print('{' + str(r_arm)[1:-1] + '},')
       print('Left arm: ')
       print('{' + str(l_arm)[1:-1] + '},')
       sample_count = sample_count +1
       raw_input('Press \'Enter\' for the next pose')


if __name__ == '__main__':
    listener()
