#!/usr/bin/env python

# visualize a DisplayRobotState with the calibration offsets applied
# author v4hn@20220817

import rospy
import re

from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayRobotState

class VisualizeCalibrationOffsets:
    def __init__(self):
        self.pub = rospy.Publisher('with_calibration_offsets', DisplayRobotState, queue_size= 1)
        self.js = {}
        self.rs = DisplayRobotState()
        self.rs.state.is_diff = True
        offsets = rospy.get_param('~offsets')
        self.offsets = { o:offsets[o] for o in offsets if re.search('_joint$', o)}
        self.sub = rospy.Subscriber('joint_states', JointState, self.cb, queue_size= 1)

    def cb(self, js_msg):
        for n,p in zip(js_msg.name, js_msg.position):
            self.js[n] = p + self.offsets.get(n, 0.0)

        self.rs.state.joint_state.name = list(self.js.keys())
        self.rs.state.joint_state.position = list(self.js.values())

        self.pub.publish(self.rs)

if __name__ == '__main__':
    rospy.init_node('visualize_calibration_offsets')
    v = VisualizeCalibrationOffsets()
    rospy.spin()
