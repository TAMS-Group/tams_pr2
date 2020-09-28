#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#define PI 3.14159265
namespace rvt = rviz_visual_tools;

class PR2TestDemo{
public:
    PR2TestDemo():visual_tools_("base_footprint")
    {
        visual_tools_.loadRemoteControl();
    }

    ~PR2TestDemo(){};

    void wrist_orientation_test(bool left_arm_, moveit::planning_interface::MoveGroupInterface& arm_)
    {
        std::vector<float> degrees{60, 30, 0, -30, -60, 0};
        geometry_msgs::Pose start_poses;

        for (auto deg: degrees) {
            geometry_msgs::Quaternion rotation;

            if (left_arm_)
            {
                tf2::Quaternion q_orig;
                q_orig.setRPY(0, 0, deg * M_PI / 180);
                tf2::convert(q_orig, rotation);

                start_poses.position.x = 0.8;
                start_poses.position.y = 0.2;
                start_poses.position.z = 1;
            }
            else
            {
                rotation = right_eef_orientation_transformation(deg);

                start_poses.position.x = 0.7;
                start_poses.position.y = -0.35;
                start_poses.position.z = 0.9;
            }
            // broadcast the tf of orientation goal
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.transform.translation.x = start_poses.position.x;
            transformStamped.transform.translation.y = start_poses.position.y;
            transformStamped.transform.translation.z = start_poses.position.z;
            transformStamped.transform.rotation.x = rotation.x;
            transformStamped.transform.rotation.y = rotation.y;
            transformStamped.transform.rotation.z = rotation.z;
            transformStamped.transform.rotation.w = rotation.w;
            transformStamped.header.frame_id = "base_footprint";
            transformStamped.child_frame_id = "orientation_goal_link";
            tf_broadcaster_.sendTransform(transformStamped);

            start_poses.orientation = rotation;
            arm_.setPoseTarget(start_poses);
            if (!(static_cast<bool>(arm_.move())))
            {
                ROS_ERROR("Arm is failed to moves to %f angle along X axis", deg);
                continue;
            }
            ROS_INFO("Arm moves to %f angle along X axis", deg);
            sleep(1.0);
        }
    }

    void left_arm_point_forward()
    {
        moveit::planning_interface::MoveGroupInterface arm("left_arm");
        std::vector<double> left_arm_pointing_forward_joints {-0.0423891, 0.0538024, 0.124877, -0.153864, 0, -0.31, -0.0};
        arm.setJointValueTarget(left_arm_pointing_forward_joints);
        if (!(static_cast<bool>(arm.move()))) {
            ROS_ERROR("Left arm is failed to POINT forward");
        }
        else
            ROS_INFO("Left arm is POINTING forward");
        sleep(1.0);
    }

    void right_arm_point_forward()
    {
        moveit::planning_interface::MoveGroupInterface arm("right_arm");
        arm.setNamedTarget("right_arm_pointing_forward");
        if (!(static_cast<bool>(arm.move()))) {
            ROS_ERROR("Right arm is failed to POINT forward");
        }
        else
            ROS_INFO("Right arm is POINTING forward");
        sleep(1.0);
    }

    void arm_circle_drawing(bool left_arm_, moveit::planning_interface::MoveGroupInterface& arm_)
    {
        ee_point_goal_ = arm_.getCurrentPose().pose;
        double angle_resolution = 0.1;
        double radius = 0.15;
        double y_center = ee_point_goal_.position.y;
        double z_center = ee_point_goal_.position.z;

        std::vector <geometry_msgs::Pose> waypoints;
        for (double angle = 0; angle <= 2 * PI; angle += angle_resolution) {
            ee_point_goal_.position.y = y_center + radius * cos(angle);
            ee_point_goal_.position.z = z_center + radius * sin(angle);
            waypoints.push_back(ee_point_goal_);
        }
        WaypointsVisualTools(waypoints);

        moveit_msgs::RobotTrajectory trajectory;
        double fraction = 0;
        for(int i=0; i<planning_attempts_, fraction < 1; i++)
        {
            fraction = arm_.computeCartesianPath(waypoints, eef_resolution_, jump_threshold_, trajectory);
            ROS_INFO("Visualizing CIRCLE path (%.2f%% acheived)", fraction * 100.0);
            if (i>0)
                ROS_INFO("The ith planning attempts");
        }
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        my_plan.trajectory_ = trajectory;
        if (!(static_cast<bool>(arm_.execute(my_plan))))
            ROS_ERROR("Arm is failed to execute CIRCLE path");
        else
            ROS_INFO("Arm moves to CIRCLE path");
        sleep(1.0);
    }

    void XYZ_motion(bool left_arm_, moveit::planning_interface::MoveGroupInterface& arm_)
    {
        if (!left_arm_)
        {
            geometry_msgs::Pose start_poses;
            start_poses.position.x = 0.7;
            start_poses.position.y = -0.4;
            start_poses.position.z = 0.8;
            start_poses.orientation = right_eef_orientation_transformation(0.0);
            arm_.setPoseTarget(start_poses);
            if (!(static_cast<bool>(arm_.move()))) {
                ROS_ERROR("Right arm is failed to moves to the starting pose of XYZ motion");
            }
            sleep(1.0);
        }

        ee_point_goal_ = arm_.getCurrentPose().pose;
        double x_center = ee_point_goal_.position.x;
        std::vector <geometry_msgs::Pose> waypoints;
        for (float i = 0; i <= 0.2; i=i+line_resolution_) {
            ee_point_goal_.position.x = x_center - i;
            waypoints.push_back(ee_point_goal_);
        }
        WaypointsVisualTools(waypoints);

        moveit_msgs::RobotTrajectory trajectory;
        double fraction = 0;
        for(int i=0; i<planning_attempts_, fraction < 1; i++)
        {
            fraction = arm_.computeCartesianPath(waypoints, eef_resolution_, jump_threshold_, trajectory);
            ROS_INFO("X path (%.2f%% acheived)", fraction * 100.0);
            if (i>0)
                ROS_INFO("The %d-th planning attempts", i);
        }

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        my_plan.trajectory_ = trajectory;
        if (!(static_cast<bool>(arm_.execute(my_plan))))
            ROS_ERROR("Arm is failed to execute along X");
        else
            ROS_INFO("Arm moves along X");

        ee_point_goal_ = arm_.getCurrentPose().pose;
        double y_center = ee_point_goal_.position.y;

        waypoints.clear();
        for (float i = 0; i <= 0.3; i=i+line_resolution_) {
            if (left_arm_)
                ee_point_goal_.position.y = y_center + i;
            else
                ee_point_goal_.position.y = y_center - i;
            waypoints.push_back(ee_point_goal_);
        }
        WaypointsVisualTools(waypoints);

        fraction = 0;
        for(int i=0; i<planning_attempts_, fraction < 1; i++)
        {
            fraction = arm_.computeCartesianPath(waypoints, eef_resolution_, jump_threshold_, trajectory);
            ROS_INFO("Y path (%.2f%% acheived)", fraction * 100.0);
            if (i>0)
                ROS_INFO("The ith planning attempts");
        }
        my_plan.trajectory_ = trajectory;
        if (!(static_cast<bool>(arm_.execute(my_plan))))
            ROS_ERROR("Arm is failed to execute along Y");
        else
            ROS_INFO("Arm moves along Y");

        ee_point_goal_ = arm_.getCurrentPose().pose;
        double z_center = ee_point_goal_.position.z;
        waypoints.clear();
        for (float i = 0; i <= 0.3; i=i+line_resolution_) {
            ee_point_goal_.position.z = z_center + i;
            waypoints.push_back(ee_point_goal_);
        }
        WaypointsVisualTools(waypoints);

        fraction = 0;
        for(int i=0; i<planning_attempts_, fraction < 1; i++)
        {
            fraction = arm_.computeCartesianPath(waypoints, eef_resolution_, jump_threshold_, trajectory);
            ROS_INFO("Z path (%.2f%% acheived)", fraction * 100.0);
            if (i>0)
                ROS_INFO("The ith planning attempts");
        }
        my_plan.trajectory_ = trajectory;
        if (!(static_cast<bool>(arm_.execute(my_plan))))
            ROS_ERROR("Arm is failed to execute along Z");
        else
            ROS_INFO("Arm moves along Z");
    }

    void diagonal_motion(bool left_arm_, moveit::planning_interface::MoveGroupInterface& arm_)
    {
        ee_point_goal_ = arm_.getCurrentPose().pose;
        double x_center = ee_point_goal_.position.x;
        double y_center = ee_point_goal_.position.y;
        double z_center = ee_point_goal_.position.z;

        std::vector <geometry_msgs::Pose> waypoints;
        for (float i = 0; i <= 0.25; i=i+line_resolution_) {
            ee_point_goal_.position.x = x_center + i;
            if (left_arm_)
                ee_point_goal_.position.y = y_center - i;
            else
                ee_point_goal_.position.y = y_center + i;
            ee_point_goal_.position.z = z_center - i;
            waypoints.push_back(ee_point_goal_);
        }
        WaypointsVisualTools(waypoints);

        moveit_msgs::RobotTrajectory trajectory;
        double fraction = 0;
        for(int i=0; i<planning_attempts_, fraction < 1; i++)
        {
            fraction = arm_.computeCartesianPath(waypoints, eef_resolution_, jump_threshold_, trajectory);
            ROS_INFO("DIALOG path (%.2f%% acheived)", fraction * 100.0);
            if (i>0)
                ROS_INFO("The ith planning attempts");
        }
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        my_plan.trajectory_ = trajectory;
        if (!(static_cast<bool>(arm_.execute(my_plan))))
            ROS_ERROR("Arm is failed to execute DIALOG path");
        else
            ROS_INFO("Arm moves along DIALOG path");
        sleep(1.0);

        visual_tools_.deleteAllMarkers();
        visual_tools_.trigger();
    }

    void left_right_accuracy_test(moveit::planning_interface::MoveGroupInterface& left_arm, moveit::planning_interface::MoveGroupInterface& right_arm)
    {
        // the right wrist should be inside of the left gripper
        std::vector<double> right_arm_cali_position {-0.24766406149, -0.0981401921335, -0.898525680844, -1.0124623003,
                                                    -0.432953648951, -0.195311464179, 0.188150321105};
        right_arm.setJointValueTarget(right_arm_cali_position);
        if (!(static_cast<bool>(right_arm.move()))) {
            ROS_ERROR("Right arm is failed to go to CALIB pose");
        }
        else
            ROS_INFO("Right arm is at CALIB pose");
        sleep(1.0);

        std::vector<double> left_arm_cali_position {0.171593096589, 0.100936643076, 0.726582097341, -1.25439410345,
                                                    1.29872764058, -0.623793745227, 2.44522417582};
        left_arm.setJointValueTarget(left_arm_cali_position);
        if (!(static_cast<bool>(left_arm.move()))) {
            ROS_ERROR("Left arm is failed to go to CALIB pose");
        }
        else
            ROS_INFO("Left arm is at CALIB pose");
        sleep(1.0);
    }


private:
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    double eef_resolution_ = 0.01;
    double jump_threshold_ = 0.0; //disable the jump threshold =0
    double line_resolution_ = 0.02;

    int planning_attempts_ = 20;
    geometry_msgs::Pose ee_point_goal_;

    moveit_visual_tools::MoveItVisualTools visual_tools_;

    void eulerToQuaternion(float eulerX, float eulerY, float eulerZ, geometry_msgs::Quaternion &q) {
        Eigen::Matrix3f rxyz, rx, ry, rz;

        rx = Eigen::AngleAxisf(eulerX, Eigen::Vector3f::UnitX());
        ry = Eigen::AngleAxisf(eulerY, Eigen::Vector3f::UnitY());
        rz = Eigen::AngleAxisf(eulerZ, Eigen::Vector3f::UnitZ());

        //Check Ordering in Axis Neuron! Here = YXZ
        rxyz = ry * rx * rz;

        Eigen::Quaternionf qf(rxyz);

        q.x = qf.x();
        q.y = qf.y();
        q.z = qf.z();
        q.w = qf.w();
    }

    void WaypointsVisualTools(std::vector <geometry_msgs::Pose> waypoints)
    {
        visual_tools_.deleteAllMarkers();
        visual_tools_.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
        for (std::size_t i = 0; i < waypoints.size(); ++i)
            visual_tools_.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
        visual_tools_.trigger();
    }

    geometry_msgs::Quaternion right_eef_orientation_transformation(float deg)
    {
        geometry_msgs::Quaternion rotation;
        // employ transformation make the eef of
        // right_arm_and_hand_ "rh_palm" and base frame "base_footprint" consistent
        tf2::Quaternion q_orig, q_rot1, q_rot2, q_new;
        q_orig.setRPY(0, 0, deg * M_PI / 180);
        q_rot1.setRPY( 0, 0, 90 * M_PI / 180);
        q_rot2.setRPY(0, 90 * M_PI / 180, 0 );
        q_new = q_rot2*q_rot1*q_orig;
        q_new.normalize();
        tf2::convert(q_new, rotation);
        return rotation;
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "pr2_demo_test");
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(3);
    spinner.start();

    bool left_arm_test;
    bool right_arm_test;
    bool left_right_accuracy_test;
    nh.param<bool>("left_arm_test", left_arm_test, false);
    nh.param<bool>("right_arm_test", right_arm_test, false);
    nh.param<bool>("left_right_accuracy_test", left_right_accuracy_test, false);

    PR2TestDemo ptd;
    moveit::planning_interface::MoveGroupInterface left_arm("left_arm");
    moveit::planning_interface::MoveGroupInterface right_arm("right_arm_and_hand");

    if (left_arm_test)
    {
        ROS_INFO("Left arm test starts");
        left_arm.setPlanningTime(15);
        ptd.left_arm_point_forward();

        ptd.wrist_orientation_test(true, left_arm);
        ptd.arm_circle_drawing(true, left_arm);
        ptd.XYZ_motion(true, left_arm);
        ptd.diagonal_motion(true, left_arm);
        ptd.left_arm_point_forward();
    }

    if (right_arm_test)
    {
        ROS_INFO("Right arm test starts");
        right_arm.setPlanningTime(15);
        ptd.right_arm_point_forward();

        ptd.wrist_orientation_test(false, right_arm);
        ptd.arm_circle_drawing(false, right_arm);
        ptd.XYZ_motion(false, right_arm);
        ptd.diagonal_motion(false, right_arm);
        ptd.right_arm_point_forward();
    }
    
    if (left_right_accuracy_test)
        ptd.left_right_accuracy_test(left_arm, right_arm);

    ros::shutdown();
    return 0;
}
