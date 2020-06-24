#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "ros/ros.h"
#include "std_msgs/UInt16.h"
 
int closeGripper()
{
  ros::NodeHandle node_handle;
  std_msgs::UInt16 msg;
  ros::Publisher gripper_angle = node_handle.advertise<std_msgs::UInt16>("gripper_angle", msg);
  msg.data = 180;
  gripper_angle.publish(msg);
} 

int openGripper()
{
  ros::NodeHandle node_handle;
  std_msgs::UInt16 msg;
  ros::Publisher gripper_angle = node_handle.advertise<std_msgs::UInt16>("gripper_angle", msg);
  msg.data = 0;
  gripper_angle.publish(msg);
} 


int main(int argc, char **argv)
{	
  ros::init(argc, argv, "move_group_1");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //----------------------------
  //Setup
  //----------------------------

  static const std::string PLANNING_GROUP = "eti_arm";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  //Using :planning_scene_interface:'PlanningSceneInterface' class to deal directly with the world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  //const robot_state::JointModelGroup *joint_model_group =
   //move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  
  move_group.setEndEffectorLink("6_Link");
  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
  // We can print the name of the reference frame for this robot.
  // also printing the current position and orientation of the robot.
  ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("robot_pose", 10);
  ROS_INFO_NAMED("eti", "x position: %f", current_pose.pose.position.x);
  ROS_INFO_NAMED("eti", "y position: %f", current_pose.pose.position.y);
  ROS_INFO_NAMED("eti", "z position: %f", current_pose.pose.position.z);
  ROS_INFO_NAMED("eti", "x orientation: %f", current_pose.pose.orientation.x);
  ROS_INFO_NAMED("eti", "y orientation: %f", current_pose.pose.orientation.y);
  ROS_INFO_NAMED("eti", "z orientation: %f", current_pose.pose.orientation.z);
  ROS_INFO_NAMED("eti", "w orientation: %f", current_pose.pose.orientation.w);


  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in Rviz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in Rviz
  visual_tools.loadRemoteControl();

  // Rviz provides many types of markers, in this demo we will use text, cylinders, and spheres
  //Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0; // above head of PR2
  visual_tools.publishText(text_pose, "MoveGroupInterface ETI Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to Rviz for large visualizations
  visual_tools.trigger();


  //-----------------------------
  //Getting Basic Information
  //-----------------------------

  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("eti", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("eti", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("eti", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the pose1");

  //-----------------------------
  //Planning to a Pose Goal
  //-----------------------------

  //Plan a motion for this group to a desired pose for end-effector
  // hardcode desired position here before running node in a separate terminal
  //set1 pose (set pose)
  geometry_msgs::Pose target_pose1;
  target_pose1.position.x = 0.000143;
  target_pose1.position.y = 0.188501;
  target_pose1.position.z = 0.553283;
  target_pose1.orientation.x = -0.711889;
  target_pose1.orientation.y = 0.000248;
  target_pose1.orientation.z = -0.003556;
  target_pose1.orientation.w = 0.702283;
  move_group.setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  //moveit::planning_interface::MoveItErrorCode success = move_group.plan(my_plan);
  //ROS_INFO_NAMED("eti", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  sleep(5.0);

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in Rviz.
  ROS_INFO_NAMED("eti", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose1 Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Execute trajectory");
  move_group.move();

  //set2 pose (ready to pick)
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue to pose2");  
  geometry_msgs::Pose target_pose2;
  target_pose2.position.x = 0.002542;
  target_pose2.position.y = 0.329424;
  target_pose2.position.z = 0.201863;
  target_pose2.orientation.x = 0.999975;
  target_pose2.orientation.y = -0.002691;
  target_pose2.orientation.z = 0.002313;
  target_pose2.orientation.w = 0.006041;
  move_group.setPoseTarget(target_pose2);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
  sleep(5.0);
  ROS_INFO_NAMED("eti", "Visualizing plan 2 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose2, "pose2");
  visual_tools.publishText(text_pose, "Pose2 Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan2.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Execute trajectory");
  move_group.move();

  //open gripper
  openGripper();

  //rostopic pub gripper_angle std/msgs/UInt16 180;

  //set3 pose (pick)
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue to pose3");  
  geometry_msgs::Pose target_pose3;
  target_pose3.position.x = 0.002943;
  target_pose3.position.y = 0.328479;
  target_pose3.position.z = 0.117504;
  target_pose3.orientation.x = 0.999975;
  target_pose3.orientation.y = -0.002825;
  target_pose3.orientation.z = 0.002234;
  target_pose3.orientation.w = 0.006046;
  move_group.setPoseTarget(target_pose3);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan3;
  sleep(5.0);
  ROS_INFO_NAMED("eti", "Visualizing plan 3 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose3, "pose3");
  visual_tools.publishText(text_pose, "Pose3 Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan3.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Execute trajectory");
  move_group.move();

  //close gripper
  closeGripper();

  //set4 pose (ready to pick)
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue to pose4");  
  geometry_msgs::Pose target_pose4;
  target_pose4.position.x = 0.002542;
  target_pose4.position.y = 0.329424;
  target_pose4.position.z = 0.201863;
  target_pose4.orientation.x = 0.999975;
  target_pose4.orientation.y = -0.002691;
  target_pose4.orientation.z = 0.002313;
  target_pose4.orientation.w = 0.006041;
  move_group.setPoseTarget(target_pose4);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan4;
  sleep(5.0);
  ROS_INFO_NAMED("eti", "Visualizing plan 4 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose4, "pose4");
  visual_tools.publishText(text_pose, "Pose4 Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan4.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Execute trajectory");
  move_group.move();

  //set5 pose (ready to place)
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue to pose5");  
  geometry_msgs::Pose target_pose5;
  target_pose5.position.x = 0.214885;
  target_pose5.position.y = 0.198713;
  target_pose5.position.z = 0.204661;
  target_pose5.orientation.x = 0.999975;
  target_pose5.orientation.y = -0.002306;
  target_pose5.orientation.z = 0.002634;
  target_pose5.orientation.w = 0.006239;
  move_group.setPoseTarget(target_pose5);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan5;
  sleep(5.0);
  ROS_INFO_NAMED("eti", "Visualizing plan 5 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose5, "pose5");
  visual_tools.publishText(text_pose, "Pose5 Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan5.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Execute trajectory");
  move_group.move();

  //set6 pose (place)
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue to pose6");  
  geometry_msgs::Pose target_pose6;
  target_pose6.position.x = 0.214885;
  target_pose6.position.y = 0.197598;
  target_pose6.position.z = 0.116915;
  target_pose6.orientation.x = 0.999975;
  target_pose6.orientation.y = -0.002329;
  target_pose6.orientation.z = 0.002639;
  target_pose6.orientation.w = 0.006289;
  move_group.setPoseTarget(target_pose6);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan6;
  sleep(5.0);
  ROS_INFO_NAMED("eti", "Visualizing plan 6 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose6, "pose6");
  visual_tools.publishText(text_pose, "Pose5 Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan6.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Execute trajectory");
  move_group.move();

  //open gripper
  openGripper();

  //set7 pose (ready to place)
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue to pose7");  
  geometry_msgs::Pose target_pose7;
  target_pose7.position.x = 0.214885;
  target_pose7.position.y = 0.198713;
  target_pose7.position.z = 0.204661;
  target_pose7.orientation.x = 0.999975;
  target_pose7.orientation.y = -0.002306;
  target_pose7.orientation.z = 0.002634;
  target_pose7.orientation.w = 0.006239;
  move_group.setPoseTarget(target_pose7);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan7;
  sleep(5.0);
  ROS_INFO_NAMED("eti", "Visualizing plan 7 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose7, "pose7");
  visual_tools.publishText(text_pose, "Pose7 Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan7.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Execute trajectory");
  move_group.move();

  //set8 pose (set pose)
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue to pose8"); 
  geometry_msgs::Pose target_pose8;
  target_pose8.position.x = 0.000143;
  target_pose8.position.y = 0.188501;
  target_pose8.position.z = 0.553283;
  target_pose8.orientation.x = -0.711889;
  target_pose8.orientation.y = 0.000248;
  target_pose8.orientation.z = -0.003556;
  target_pose8.orientation.w = 0.702283;
  move_group.setPoseTarget(target_pose8);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan8;
  sleep(5.0);
  ROS_INFO_NAMED("eti", "Visualizing plan 8 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose8, "pose8");
  visual_tools.publishText(text_pose, "Pose8 Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan8.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Execute trajectory");
  move_group.move();
  
  //set9 pose (home)
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue to pose9"); 
  geometry_msgs::Pose target_pose9;
  target_pose9.position.x = -0.000819;
  target_pose9.position.y = 0.001800;
  target_pose9.position.z = 0.892950;
  target_pose9.orientation.x = -0.000076;
  target_pose9.orientation.y = 0.000000;
  target_pose9.orientation.z = -0.000022;
  target_pose9.orientation.w = 1.000000;
  move_group.setPoseTarget(target_pose9);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan9;
  sleep(5.0);
  ROS_INFO_NAMED("eti", "Visualizing plan 9 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose9, "pose9");
  visual_tools.publishText(text_pose, "Pose9 Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan9.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Execute trajectory");
  move_group.move();
  
  ros::shutdown();  
  return 0;
}
