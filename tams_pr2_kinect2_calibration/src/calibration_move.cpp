/*TODO
  - kinect ansprechen
    - calibration node starten und schliessen
    - calibration_control messages publizieren
  - move torso down when tuck_arms is working again
*/

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>

//moveit includes
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>

//visualization includes
#include <visualization_msgs/Marker.h>

//msgs include
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/String.h"

//action client
#include <actionlib/client/simple_action_client.h>
//point head action
#include <pr2_controllers_msgs/PointHeadAction.h> 
//move gripper action
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
//move torso action
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
//calibration control service
#include <tams_pr2_kinect2_calibration/Kinect2CalibrationControl.h>

#include <tf/transform_datatypes.h>

class CalibrationMove {
private:
  actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction>* point_head_client_;
  actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction>* l_gripper_command_client_;
  actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction>* torso_client_;

  ros::NodeHandle nh;
  ros::Publisher marker_pub;
  visualization_msgs::Marker move_space;
  moveit::planning_interface::MoveGroupInterface left_arm;
  moveit::planning_interface::MoveGroupInterface right_arm;

  geometry_msgs::PointStamped head_point;
  ros::ServiceClient controlService;

public:

  CalibrationMove():left_arm("left_arm"),right_arm("right_arm") {
    // Advertise visualization marker
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    
    // Intialize random seed
    srand(time(0));
    
    // Initalize and wait for point_head action
    point_head_client_ = new actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction>("/head_traj_controller/point_head_action", true);
    while(!point_head_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the head_traj_controller/point_head_action server to come up");
    }
    
    // Initialize and wait for l_gripper action
    l_gripper_command_client_ = new actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction>("/l_gripper_controller/gripper_action", true);
    while(!l_gripper_command_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the torso_controller/position_joint_action server to come up");
    }

    // Initialize and wait for torso action
    torso_client_ = new actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction>("/torso_controller/position_joint_action", true);
    while(!torso_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the l_gripper_controller/gripper_action server to come up");
    }
  
    // Initialize publisher for controlling the kinect2 calibration node and the two messages required
    controlService = nh.serviceClient<tams_pr2_kinect2_calibration::Kinect2CalibrationControl>("/kinect2_calibration_control", 10);

    // Set center of move space and for head point
    head_point.header.frame_id = "/torso_lift_link";
    head_point.point.x = 0.7;
    head_point.point.y = 0.15;
    head_point.point.z = 0.2;
  }

  ~CalibrationMove() {
    delete point_head_client_;
    delete l_gripper_command_client_;
    delete torso_client_;
  }

  // returns random number between -1.0 and +1.0
  double random_dev() {
    return ((double) rand() / (RAND_MAX)*2)-1;
  }

  // Open the gripper
  void gripper_open(){
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = 0.08;
    open.command.max_effort = -1.0;  // Do not limit effort (negative)
    
    ROS_INFO("Sending open goal");
    l_gripper_command_client_->sendGoal(open);
    l_gripper_command_client_->waitForResult();
    if(l_gripper_command_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper opened!");
    else
      ROS_INFO("The gripper failed to open.");
  }
  
  // Close the gripper
  void gripper_close(){
    pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
    squeeze.command.position = 0.0;
    squeeze.command.max_effort = 50.0;  // Close gently
    
    ROS_INFO("Sending squeeze goal");
    l_gripper_command_client_->sendGoal(squeeze);
    l_gripper_command_client_->waitForResult();
    if(l_gripper_command_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper closed!");
    else
      ROS_INFO("The gripper failed to close.");
  }

  // Point Head at move space center
  void point_head() {
    pr2_controllers_msgs::PointHeadGoal head_goal;
    geometry_msgs::PointStamped tmp = head_point;
    tmp.point.z = 0.0;
    head_goal.target = tmp;
    head_goal.min_duration = ros::Duration(0.5);
    head_goal.max_velocity = 1.0;
    point_head_client_->sendGoal(head_goal);
    point_head_client_->waitForResult(ros::Duration(2));
  }

  // Set move space and publish visualization marker
  void show_move_space_marker() {
    move_space.header.frame_id = "/torso_lift_link";
    move_space.ns = "move_space";
    move_space.type = visualization_msgs::Marker::CUBE;
    move_space.action = visualization_msgs::Marker::ADD;
    move_space.color.g = 1.0f;
    move_space.color.a = 0.3;

    move_space.pose.position = head_point.point;
    move_space.pose.orientation.x = 0.0;
    move_space.pose.orientation.y = 0.0;
    move_space.pose.orientation.z = 0.0;
    move_space.pose.orientation.w = 1.0;
    //values incorrect
    move_space.scale.x = 0.6;
    move_space.scale.y = 0.8;
    move_space.scale.z = 0.8;
    marker_pub.publish(move_space);
  }

  // Move arm to random pose
  bool move_arm(bool randomize=true) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "torso_lift_link";
    if (randomize) {
      pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw((0.5*M_PI)+(0.16*M_PI)*random_dev(), (-0.5*M_PI)+(0.16*M_PI)*random_dev(), (0.16*M_PI)*random_dev());
      pose.pose.position.x = head_point.point.x+0.25*random_dev();
      // -0.15 as the camera lens is not centered on the kinect v2
      pose.pose.position.y = head_point.point.y-0.3+0.65*random_dev();
      // 0.15 as the camera is mounted on top of the head and the pointing direction corresponds to the 'spider-eye' cameras
      pose.pose.position.z = head_point.point.z+0.15+0.3*random_dev();
    } else {
      pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw((0.5*M_PI), (-0.5*M_PI), 0);
      pose.pose.position.x = head_point.point.x;
      pose.pose.position.y = head_point.point.y;
      pose.pose.position.z = head_point.point.z;
    }

    left_arm.clearPoseTargets();
    left_arm.clearPathConstraints();

    left_arm.setPlanningTime(0.5);
    left_arm.setNumPlanningAttempts(10);
    
    left_arm.setPoseTarget(pose);
    return bool(left_arm.move());
  }

  // Add calibration pattern to move group for collision checking
  void add_pattern_collision() {
    ros::ServiceClient apply_planning_scene_client = nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    apply_planning_scene_client.waitForExistence();

    moveit_msgs::ApplyPlanningScene srv;
    moveit_msgs::PlanningScene planning_scene;

    moveit_msgs::CollisionObject pattern;
    pattern.header.frame_id = "l_gripper_tool_frame";
    pattern.id = "pattern";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.21;
    primitive.dimensions[1] = 0.005;
    primitive.dimensions[2] = 0.20;
    pattern.primitives.push_back(primitive);

    geometry_msgs::Pose pattern_pose;
    pattern_pose.orientation.w = 1.0;
    pattern_pose.position.x = 0.1;
    pattern.primitive_poses.push_back(pattern_pose);

    pattern.operation = pattern.ADD;
    planning_scene.world.collision_objects.push_back(pattern);
    planning_scene.is_diff = true;
    srv.request.scene = planning_scene;
    apply_planning_scene_client.call(srv);

    std::vector<std::string> allowed_collision_links;
    allowed_collision_links.push_back("l_gripper_r_finger_link");
    allowed_collision_links.push_back("l_gripper_l_finger_link");
    allowed_collision_links.push_back("l_gripper_r_finger_tip_link");
    allowed_collision_links.push_back("l_gripper_l_finger_tip_link");

    left_arm.attachObject("pattern", "", allowed_collision_links);
  }

  // Move the right arm to the side
  bool move_right_arm_to_side() {
    right_arm.setNamedTarget("right_arm_to_side");
    return bool(right_arm.move());
  }

  // Move the torso to target position, all the way up is 0.325, down is 0.0115
  // use 0.3 and 0.02 for normal operation (min height due to tray)
  void move_torso(float goal_height){
    pr2_controllers_msgs::SingleJointPositionGoal goal;
    goal.position = goal_height;
    goal.min_duration = ros::Duration(2.0);
    goal.max_velocity = 1.0;
    
    ROS_INFO("Sending torso goal to %f",goal_height);
    torso_client_->sendGoal(goal);
    torso_client_->waitForResult();
  }

  bool control_request(bool save, bool quit) {
    tams_pr2_kinect2_calibration::Kinect2CalibrationControl srv;
    srv.request.save = save;
    srv.request.quit = quit;
    if (controlService.call(srv)) {
      if (quit && srv.response.suceeded) {
        ROS_INFO("Exiting calibration node");
        return true;
      } else if (save && srv.response.suceeded) {
        ROS_INFO("Recorded calibration image");
        return true;
      } else if (!srv.response.suceeded) {
        ROS_WARN("Service call to kinect2_calibration_control returned a failure");
        return false;
      }
    } else {
      ROS_ERROR("Failed to call service kinect2_calibration_control");
    }
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "calibrationMove");

  ros::AsyncSpinner spinner(3);
  spinner.start();

  CalibrationMove calibrationMove;
  //calibrationMove.show_move_space_marker();

  // Wait to proceed
  std::cout << "Press <enter> to move robot to the initial pose.";
  std::string throwaway;
  std::getline(std::cin, throwaway);

  //Move torso to upper position
  calibrationMove.move_torso(0.1);

  // Point head at center of move space
  calibrationMove.point_head();

  // Move right arm to side
  if (!calibrationMove.move_right_arm_to_side()) {
    ROS_ERROR("Failed to move right arm out of the way.");
    return -1;
  }

  // Move left arm to center of move space
  if (!calibrationMove.move_arm(false)) {
    ROS_ERROR("Failed to move left arm to center of move space.");
    return -1;
  }

  // Open gripper, wait for pattern, add pattern collision and close gripper
  std::cout << "Please put the checkerboard in the left hand (open/close the gripper with the joystick's left/right D-Pad buttons). Press <enter> to move to the first pose.";
  std::getline(std::cin, throwaway);

  int image_count = 0;
  int target_image_count = 100;
  //ROS_INFO("Current Image Count: %d",image_count);
  calibrationMove.add_pattern_collision();
  //if (calibrationMove.control_request(true, false)) { image_count++; }
  //ROS_INFO("Current Image Count: %d",image_count);
  //ros::Duration(0.5).sleep();

  ros::NodeHandle nh("~");
  bool demo;
  nh.param<bool>("demo", demo, false);

  if(!demo) {
    while (image_count < target_image_count && ros::ok()) {
      if(calibrationMove.move_arm()) {
        ROS_INFO("Moved left arm, try to save calibration image");
        ros::Duration(2.0).sleep();
        if (calibrationMove.control_request(true, false))
          image_count++;
        ROS_INFO("Current Image Count: %d / %d",image_count, target_image_count);
        ros::Duration(0.5).sleep();
      } //else {
        //ROS_WARN("Could not reach random pose, trying another random pose");
     // }
    }
  } else {
    while(ros::ok()) {
      calibrationMove.move_arm();
      ros::Duration(2.0).sleep();
    }
  }


  // Move left arm to center of move space
  if (!calibrationMove.move_arm(false)) {
    ROS_ERROR("Failed to move left arm to center of move space.");
    return -1;
  }

  // Let go of the pattern
  ROS_INFO("Calibration complete!");
  
  // Exit the calibration node
  calibrationMove.control_request(false, true);

  spinner.stop();
  return 0;
}
