#include <ros/ros.h>
//#include <move_group.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>

using namespace ros;
using namespace moveit;


/* This program will move the robot to it's desired pose(s)
   It's using the moveIt package irb120_moveit
   */
int main(int argc, char **argv){
   init(argc, argv, "movement");
   NodeHandle n;

   // gee, it would be handy if this were in the FUCKING TUTORIAL
   AsyncSpinner spinner(1);
   spinner.start();


   planning_interface::MoveGroupInterface group("manipulator");
   planning_interface::PlanningSceneInterface planning_scene_interface;

   Publisher display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
   moveit_msgs::DisplayTrajectory display_trajectory;

   ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
   ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

   geometry_msgs::Pose target_pose1;
   target_pose1.orientation.w = -0.5;
   /*target_pose1.orientation.x =  0.5;
   target_pose1.orientation.y = -0.5;
   target_pose1.orientation.z =  0.5; */
   target_pose1.position.x = 0.374;
   target_pose1.position.y = 0.00;
   target_pose1.position.z = 0.0;
   group.setPoseTarget(target_pose1);

   moveit::planning_interface::MoveGroupInterface::Plan my_plan;
   bool success = group.plan(my_plan);

   ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
   /* Sleep to give Rviz time to visualize the plan. */
   sleep(5.0);

   std::vector<double> group_variable_values;
   group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

   group_variable_values[0] = -1.0;
   group.setJointValueTarget(group_variable_values);
   success = group.plan(my_plan);

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
   group.setPathConstraints(test_constraints);
   group.setPoseTarget(target_pose1);

   success = group.plan(my_plan);

   // let's never speak of this again
   group.clearPathConstraints();

   ROS_INFO("Visualizing plan 3 (constraints) %s",success?"":"FAILED");
   /* Sleep to give Rviz time to visualize the plan. */
   sleep(10.0);

   // Linear paths are specified using waypoints
   // make waypoints
   std::vector<geometry_msgs::Pose> waypoints;

   geometry_msgs::PoseStamped temp = group.getCurrentPose("tool0");
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
   double fraction = group.computeCartesianPath(waypoints,
                                                0.01,  // eef_step
                                                0.0,   // jump_threshold
                                                trajectory);

   ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
               fraction * 100.0);
   /* Sleep to give Rviz time to visualize the plan. */
   sleep(15.0);

   moveit_msgs::CollisionObject collision_object;
   collision_object.header.frame_id = group.getPlanningFrame();

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

   group.setPlanningTime(10.0);

   group.setStartState(*group.getCurrentState());
   group.setPoseTarget(target_pose1);
   success = group.plan(my_plan);

   ROS_INFO("Visualizing plan 5 (pose goal move around box) %s",
           success?"":"FAILED");
   /* Sleep to give Rviz time to visualize the plan. */
   sleep(10.0);

   // Attach to robot
   ROS_INFO("Attach the object to the robot");
   group.attachObject(collision_object.id);
   /* Sleep to give Rviz time to show the object attached (different color). */
   sleep(4.0);

   ROS_INFO("Detach the object from the robot");
   group.detachObject(collision_object.id);
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
