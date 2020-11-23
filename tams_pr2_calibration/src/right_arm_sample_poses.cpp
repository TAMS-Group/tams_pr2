#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/ApplyPlanningScene.h>

std::vector<std::vector<double>> head_sample_poses{
  {0.03765960482173557, 0.8584893731655178},
  {-1.0047208376393577, 0.47950857938746905},
  {-1.0051397166598366, 0.5093537095965721},
  {-1.7429951112329591, 0.8367076641006285},
  {-2.0202930227898186, 0.68235074505425},

  {-1.3959538427664069, 1.1251058697001715},
  {-0.7866943074802261, 1.1160999707598809},
  {-0.35849522879593737, 1.256948041395823},
  {-0.337970156792484, 0.9579731405291946},
  {-0.22895689171291822, 1.3133919894053199},

  {-0.00820764792067542, 1.3252253217338414},
  {-0.007788768900196763, 1.3252253217338414},
  {-0.008102928165555738, 1.3252253217338414},
  {-0.9204214347680317, 1.074840387242735},
  {-1.381083637539415, 0.9175513150530059}
};

std::vector<std::vector<double>> r_arm_sample_poses{
  {0.017054986001265204, -0.19422994344782765, 0.11437254353113424, -1.2680198678465777, 1.067452295498693},
  {-0.5642862149059189, -0.33905645180265426, 0.634723955101415, -1.4310316888644299, 1.1571731273590122},
  {-0.9753372237618544, -0.35386056334593385, 0.3423971066291125, -0.8583187546311941, 1.263785308608895},
  {-1.5549373857045419, -0.2633439956241674, 0.14900918602702973, -1.0114861672749664, 1.33447444886248},
  {-1.9772636945210273, -0.27848648685986466, 0.6661535010699131, -1.1574150443495625, 1.33343320129737},

  {-1.7564833724993743, -0.12443913188665269, -0.8469546405563336, -1.425530401831856, 1.8203899792471883},
  {-0.8266027522234219, 0.25970641393067595, -1.087647234937533, -1.36906982439228, 1.823340180681667},
  {-0.7820818987027206, 0.2998890024052919, -0.6094691427025322, -1.3599492695751179, 1.8241500398989747},
  {-0.13648806934387225, -0.08848628956725954, 0.33694504253253643, -0.7287489679429368, 1.8034986298576245},
  {-0.11227933707563253, 0.6737985625269816, 0.14836776672155016, -1.2342882920942158, 1.803556476944575},

  {-0.19062609047798407, 0.07376677294708447, -1.2919392837327681, -1.291183181667942, 2.0469191717455835},
  {-0.1730498875983031, 0.2458328465415454, -1.1054466206645903, -1.0187247028441428, 2.6122009054264606},
  {-0.1432864119671452, 0.8208245388825237, -1.3753237934451088, -1.2273392979478064, 2.6149197185131365},
  {-0.7632620965626847, 0.2369503796155777, 0.5877399909750385, -1.0902414342676057, 1.181526750965198},
  {-2.0981415426275127, -0.03637581693491537, -0.8995510236056563, -1.428281045348143, 2.0624221910483334}
};

void attachCheckerboard() {
  ros::NodeHandle nh;

  ros::ServiceClient planning_scene_diff_client = nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
  planning_scene_diff_client.waitForExistence();

  moveit_msgs::ApplyPlanningScene srv;
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.is_diff = true;
  planning_scene.robot_state.is_diff = true;

  moveit_msgs::AttachedCollisionObject object;

  object.link_name = "r_forearm_roll_link";
  object.object.header.frame_id = "r_forearm_roll_link";
  object.object.id = "checkerboard";

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.21;
  primitive.dimensions[1] = 0.005;
  primitive.dimensions[2] = 0.2;

  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  pose.position.x = 0.22;
  pose.position.y = 0.07;
  pose.position.z = 0.0;

  object.object.primitives.push_back(primitive);
  object.object.primitive_poses.push_back(pose);

  object.object.operation = object.object.ADD;

  object.touch_links.push_back("rh_forearm");
  object.touch_links.push_back("rh_adapter_link");

  planning_scene.robot_state.attached_collision_objects.push_back(object);

  srv.request.scene = planning_scene;
  planning_scene_diff_client.call(srv);
};



int main(int argc, char** argv){
  ros::init(argc, argv, "samplePoses");

  ros::AsyncSpinner spinner(1);
  spinner.start();


  attachCheckerboard();

  moveit::planning_interface::MoveGroupInterface left_arm("left_arm");
  left_arm.setMaxVelocityScalingFactor(.1);

  moveit::planning_interface::MoveGroupInterface right_arm("right_arm");
  left_arm.setMaxVelocityScalingFactor(.1);

  moveit::planning_interface::MoveGroupInterface head("head");
  head.setMaxVelocityScalingFactor(.1);

  // Wait to proceed
  std::cout << "--------------------------------------------------------------------------" << std::endl;
  std::cout << "Press <enter> to move to the initial pose with the right and the left arm.";
  std::string throwaway;
  std::getline(std::cin, throwaway);

  {
    moveit::planning_interface::MoveGroupInterface group("left_arm");
    group.setNamedTarget("left_arm_to_side");
    while(!group.move() and ros::ok());
  }
  right_arm.setJointValueTarget(r_arm_sample_poses[0]);
  while(!right_arm.move() and ros::ok());

  head.setJointValueTarget(head_sample_poses[0]);
  while(!head.move() and ros::ok());

  // Wait to proceed
  std::cout << "Please attach the checkerboard to the Shadow Hand. Press <enter> to move to the first pose.";
  std::getline(std::cin, throwaway);

  for (size_t i = 1; i < head_sample_poses.size(); ++i) {
    if(!ros::ok())
      return 0;

    ROS_INFO_STREAM("Pose: " << i << "/" << head_sample_poses.size()-1);
    right_arm.setJointValueTarget(r_arm_sample_poses[i]);
    right_arm.move();

    head.setJointValueTarget(head_sample_poses[i]);
    head.move();

    // Wait to proceed
    std::cout << "Press enter to continue...";
    std::getline(std::cin, throwaway);
 }

  return 0;
}
