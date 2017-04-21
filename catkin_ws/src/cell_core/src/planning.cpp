#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayTrajectory.h>


using namespace ros;
using namespace moveit;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planning");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Setting up movegroup named "manipulator"
  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointer refering to planning group for better performance
  const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  // Getting Basic Information
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
  
  // Defining target_pose1
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = 1.0;
  target_pose1.orientation.y = 0.0;
  target_pose1.orientation.z = 0.0;
  target_pose1.orientation.w = 0.0;
  target_pose1.position.x = 1.2;
  target_pose1.position.y = 0.7;
  move_group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = move_group.plan(my_plan);

   ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
   /* Sleep to give Rviz time to visualize the plan. */
   sleep(5.0);

   std::vector<double> group_variable_values;
   move_group.getCurrentState()->copyJointGroupPositions(move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()), group_variable_values);

   group_variable_values[0] = -1.0;
   move_group.setJointValueTarget(group_variable_values);
   success = move_group.plan(my_plan);

   ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
   /* Sleep to give Rviz time to visualize the plan. */
   sleep(5.0);

   // Doing this with path constraints (?)
   // Setting orientation constraint
   moveit_msgs::OrientationConstraint ocm;
   ocm.link_name = "link_6";
   ocm.header.frame_id = "base_link";
   ocm.orientation.w = -0.5;
   ocm.orientation.x =  0.5;
   ocm.orientation.y = -0.5;
   ocm.orientation.z =  0.5;
   ocm.absolute_x_axis_tolerance = 0.1;
   ocm.absolute_y_axis_tolerance = 0.1;
   ocm.absolute_z_axis_tolerance = 0.1;
   ocm.weight = 1.0;
   // add to constraint list
   moveit_msgs::Constraints test_constraints;
   test_constraints.orientation_constraints.push_back(ocm);
   // Using previous joint target
   move_group.setPathConstraints(test_constraints);
   move_group.setPoseTarget(target_pose1);

   success = move_group.plan(my_plan);

   // let's never speak of this again
   move_group.clearPathConstraints();

   ROS_INFO("Visualizing plan 3 (constraints) %s",success?"":"FAILED");
   /* Sleep to give Rviz time to visualize the plan. */
   sleep(10.0);

   // Linear paths are specified using waypoints
   // make waypoints
   std::vector<geometry_msgs::Pose> waypoints;

   geometry_msgs::PoseStamped temp = move_group.getCurrentPose("tool0");
   geometry_msgs::Pose target_pose3 = temp.pose;
   target_pose3.position.x -= 0.2;
   target_pose3.position.z -= 0.2;
   waypoints.push_back(target_pose3);  // up and out

   target_pose3.position.y -= 0.2;
   waypoints.push_back(target_pose3);  // left

   target_pose3.position.z += 0.2;
   target_pose3.position.y += 0.2;
   target_pose3.position.x += 0.2;
   waypoints.push_back(target_pose3);  // down and right (back to start)

   // Make path with points at .01m steps
   moveit_msgs::RobotTrajectory trajectory;
   double fraction = move_group.computeCartesianPath(waypoints,
                                                0.01,  // eef_step
                                                0.0,   // jump_threshold
                                                trajectory);

   ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
               fraction * 100.0);
   /* Sleep to give Rviz time to visualize the plan. */
   sleep(15.0);

   moveit_msgs::CollisionObject collision_object;
   collision_object.header.frame_id = move_group.getPlanningFrame();

   /* The id of the object is used to identify it. */
   collision_object.id = "box1";

   /* Define a box to add to the world. */
   shape_msgs::SolidPrimitive primitive;
   primitive.type = primitive.BOX;
   primitive.dimensions.resize(3);
   primitive.dimensions[0] = 0.4;
   primitive.dimensions[1] = 0.1;
   primitive.dimensions[2] = 0.4;

   /* A pose for the box (specified relative to frame_id) */
   geometry_msgs::Pose box_pose;
   box_pose.orientation.w = 1.0;
   box_pose.position.x =  0.6;
   box_pose.position.y = -0.4;
   box_pose.position.z =  1.2;

   collision_object.primitives.push_back(primitive);
   collision_object.primitive_poses.push_back(box_pose);
   collision_object.operation = collision_object.ADD;

   std::vector<moveit_msgs::CollisionObject> collision_objects;
   // add to list of collision objects
   collision_objects.push_back(collision_object);
   ROS_INFO("Add an object into the world");
   planning_scene_interface.addCollisionObjects(collision_objects);

   /* Sleep so we have time to see the object in RViz */
   sleep(2.0);

   move_group.setPlanningTime(10.0);

   move_group.setStartState(*move_group.getCurrentState());
   move_group.setPoseTarget(target_pose1);
   success = move_group.plan(my_plan);

   ROS_INFO("Visualizing plan 5 (pose goal move around box) %s",
           success?"":"FAILED");
   /* Sleep to give Rviz time to visualize the plan. */
   sleep(10.0);

   // Attach to robot
   ROS_INFO("Attach the object to the robot");
   move_group.attachObject(collision_object.id);
   /* Sleep to give Rviz time to show the object attached (different color). */
   sleep(4.0);

   ROS_INFO("Detach the object from the robot");
   move_group.detachObject(collision_object.id);
   /* Sleep to give Rviz time to show the object detached. */
   sleep(4.0);

   ROS_INFO("Remove the object from the world");
   std::vector<std::string> object_ids;
   object_ids.push_back(collision_object.id);
   planning_scene_interface.removeCollisionObjects(object_ids);
   /* Sleep to give Rviz time to show the object is no longer there. */
   sleep(4.0);


   spinner.stop();
   return(0);
}