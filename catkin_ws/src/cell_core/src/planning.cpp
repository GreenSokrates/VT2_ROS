#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planning");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "irb120_tcp";

  // Setting up movegroup
  moveit::planning_interface::MoveGroupInterface move_group("PLANNING_GROUP");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // Raw pointer refering to planning group for better performance
  const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  
  // Defining target_pose1
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 0.0;
  target_pose1.position.x = 0.0;
  target_pose1.position.y = 0.0;
  target_pose1.position.z = 1.0;
  move_group.setPoseTarget(target_pose1);

  // Call the Planner, compute and visualize
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = move_group.plan(my_plan);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Uncomment below line when working with a real robot
  move_group.move();

  ros::shutdown();
}