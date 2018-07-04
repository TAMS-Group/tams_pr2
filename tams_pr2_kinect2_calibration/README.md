# tams_pr2_kinect2_calibration

This tool calibrates only the intrinsics of the head mounted kinect2 sensor.

-----
### Usage
Create the following folder: ~/kinect_cal_data on the pr2-head. the calibration images and yaml files will be saved here.

Before starting the calibration start the full robot with the shadow hand as described in the tams_pr2 or tams_pr2_bringup package. Also launch the move group.

After the bringup, launch the following files in the correct order one after another and follow the instructions on the terminal:
```
roslaunch tams_pr2_kinect2_calibration kinect2_calibration_steps.launch record:=true type:=color
roslaunch tams_pr2_kinect2_calibration kinect2_calibration_steps.launch record:=false type:=color
roslaunch tams_pr2_kinect2_calibration kinect2_calibration_steps.launch record:=true type:=ir
roslaunch tams_pr2_kinect2_calibration kinect2_calibration_steps.launch record:=false type:=ir
roslaunch tams_pr2_kinect2_calibration kinect2_calibration_steps.launch record:=true type:=sync
roslaunch tams_pr2_kinect2_calibration kinect2_calibration_steps.launch record:=false type:=sync
roslaunch tams_pr2_kinect2_calibration kinect2_calibration_steps.launch record:=false type:=depth
```
If you have broken images during the calibration (you get an error message), just remove them from the ~/kinect_cal_data.

Find out the serial number of your kinect2 by looking at the first lines printed out by the kinect2_bridge. The line looks like this: device serial: 012526541941

Create the calibration results directory in kinect2_bridge/data/$serial: roscd kinect2_bridge/data; mkdir 012526541941

Copy the following files from your calibration directory (~/kinect_cal_data) into the directory you just created: calib_color.yaml calib_depth.yaml calib_ir.yaml calib_pose.yaml

Restart the kinect2_bridge and be amazed at the better data.
    
