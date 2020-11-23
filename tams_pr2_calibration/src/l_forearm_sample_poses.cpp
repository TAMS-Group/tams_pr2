#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>


std::vector<std::vector<double>> wrist_sample_poses{{-1.731793367204064, 1.744857661537038}, {-1.7364488216547214, 1.551373493854142}, {-1.6834984658935146, 1.1645356852486484}, {-1.377891811113964, 1.1695827199802014}, {-1.4705223020059999, 1.8755149485961204}, {-1.4618205179860801, 1.1010561708233397}, {-1.4462878335105254, 0.980231899706766}, {-1.6510843204193169, 1.1202871135073607}, {-1.649909579576627, 0.9603048143011514}, {-1.6732303607500103, 1.5566380731861928}, {-1.6175824519426296, 2.2055736164716437}, {-1.5679387741089919, 2.1727243817964492}};

int main(int argc, char** argv){
  ros::init(argc, argv, "samplePoses");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface left_arm("left_arm");
  left_arm.setMaxVelocityScalingFactor(.1);

  std::vector<double> pose = left_arm.getCurrentJointValues();

  // Wait to proceed
  std::cout << "Please put the board in the left hand (open/close the gripper with the joystick's left/right D-Pad buttons). Press <enter> to move to the first pose.";
  std::string throwaway;
  std::getline(std::cin, throwaway);

  for (size_t i = 0; i < wrist_sample_poses.size(); ++i) {
    if(!ros::ok())
      return 0;

    ROS_INFO_STREAM("Pose: " << i << "/" << wrist_sample_poses.size()-1);

    pose[5] = wrist_sample_poses[i][0];
    pose[6] = wrist_sample_poses[i][1];
    left_arm.setJointValueTarget(pose);
    left_arm.move();

    // Wait to proceed
    std::cout << "Press enter to continue...";
    std::getline(std::cin, throwaway);
 }

  return 0;
}
