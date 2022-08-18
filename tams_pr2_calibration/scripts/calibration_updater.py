#!/usr/bin/env python

import rospy
from ruamel.yaml import YAML
from ruamel.yaml.scalarfloat import ScalarFloat

import re

def main():
    calibration_file = rospy.get_param('~calibration_file')

    yaml= YAML()
    cal = yaml.load(open(calibration_file).read())
    
    offsets= rospy.get_param('~offsets')

    calibration_keys = list(cal.keys())
    for k in calibration_keys:
        m = re.match("(.*)_calib_ref$", k)
        if m:
            joint = m.group(1)+"_joint"
            if joint in offsets:
                cal[k] += offsets[joint]
            continue
        m = re.match("(.*)_pos_([xyz])$", k)
        if m:
            joint= m.group(1)+"_joint_"+m.group(2)
            if joint in offsets:
                cal[k] += offsets[joint]
            continue
        m = re.match("(.*)_rot_([rpy])$", k)
        if m:
            rospy.logerr('cannot add rotation offsets to calibration config yet')
            # TODO: check that only a OR b OR c are specified in the offsets.
            # In that case a -> r, b -> p, c -> y and can be added directly
            # If all three are specified they represent A/B/C species a *magnitude coded AngleAxis representation*
            # I believe the correct way to apply them is to apply them is to convert the current calibration to the same representation and add them there.
            # Two out of three are a fatal error condition as calibration does not make sense there due to orientation coupling
            # -- v4hn@20220818
            continue
        rospy.logerr("cannot handle calibration variable '{}'. Ignoring.".format(k))

    yaml.dump(cal, open(calibration_file, 'w'))

if __name__ == '__main__':
  rospy.init_node('calibration_updater')
  main()
