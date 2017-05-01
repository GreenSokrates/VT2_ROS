#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>

using namespace ros;
using namespace moveit;

int main(int argc, char **argv){
   init(argc, argv, "movement");
   NodeHandle n;

   AsyncSpinner spinner(1);
   spinner.start();

   // Setup of MoveGroupInterface and PlanningSceneInterface
   planning_interface::MoveGroupInterface group("manipulator");
   planning_interface::PlanningSceneInterface planning_scene_interface;
   Publisher display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
   moveit_msgs::DisplayTrajectory display_trajectory;

   // Getting basic Infos from Robot
   ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
   ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

   // Initialising Pose Named 'pose1'
   geometry_msgs::Pose pose1;
   pose1.position.x = 0.15;
   pose1.position.y = 0.15;
   pose1.position.z = 0.05;
   pose1.orientation.w = 0.707;
   pose1.orientation.x = 0.0;
   pose1.orientation.y = 0.0;
   pose1.orientation.z = 0.707;

   group.setPoseTarget(pose1);

   // Construction of planner
   moveit::planning_interface::MoveGroupInterface::Plan my_plan;

   bool success = group.plan(my_plan);
   ROS_INFO("Planning of Plan 1: %s", success?"Succeded":"FAILED");
   sleep(2.0);
   // Execute movement if Planning succeded
   if(success) group.move();
   sleep(5.0);

   geometry_msgs::Pose pose2;
   pose2.position.x = -0.2;
   pose2.position.y = -0.2;
   pose2.position.z =  0.5;
   pose2.orientation.w = 0.707;
   pose2.orientation.x = 0;
   pose2.orientation.y = 0.707;
   pose2.orientation.z = 0;

   group.setPoseTarget(pose2);
   success = group.plan(my_plan);
   ROS_INFO("Planning of Plan 2: %s", success?"Succeded":"FAILED");
   sleep(2.0);
   // Execute movement if planning succeded
   if(success) group.move();

   // Attatching Object to PlanningSceneInterface
   moveit_msgs::CollisionObject collision_object;
   collision_object.header.frame_id = group.getPlanningFrame();

   collision_object.id = "box1";

   shape_msgs::SolidPrimitive primitive;
   primitive.type = primitive.BOX;
   primitive.dimensions.resize(3);
   primitive.dimensions[0] = 0.8;
   primitive.dimensions[1] = 0.8;
   primitive.dimensions[2] = 0.1;

   /* A pose for the box (specified relative to frame_id) */
   geometry_msgs::Pose box_pose;
   box_pose.orientation.w = 1.0;
   box_pose.position.x =  0.0;
   box_pose.position.y =  0.0;
   box_pose.position.z =  1.1;

   collision_object.primitives.push_back(primitive);
   collision_object.primitive_poses.push_back(box_pose);
   collision_object.operation = collision_object.ADD;

   std::vector<moveit_msgs::CollisionObject> collision_objects;

   // add list of collision collision_objects
   collision_objects.push_back(collision_object);
   ROS_INFO("Add an object into the world");
   planning_scene_interface.addCollisionObjects(collision_objects);
   sleep(2.0);

   group.setPlanningTime(10.0);

   geometry_msgs::Pose pose3;
   pose3.position.x = -0.2;
   pose3.position.y = 0.2;
   pose3.position.z = 0.25;
   pose3.orientation.w = 0;
   pose3.orientation.x = 1;
   pose3.orientation.y = 0;
   pose3.orientation.z = 0;

   group.setPoseTarget(pose3);
   success = group.plan(my_plan);
   ROS_INFO("Planning of Plan 3: %s", success?"Succeded":"FAILED");
   sleep(2.0);
   // Execute movement if Planning succeded
   if(success) group.move();

   sleep(5.0);

   // Linear paths

   std::vector<geometry_msgs::Pose> waypoints;
   geometry_msgs::PoseStamped temp = group.getCurrentPose("tool0");
   geometry_msgs::Pose pose4 = temp.pose;
   pose4.position.x -= 0.1;
   pose4.position.z -= 0.15;
   waypoints.push_back(pose4);  // up and out

   pose4.position.y -= 0.1;
   waypoints.push_back(pose4);  // left

   pose4.position.z += 0.15;
   pose4.position.y += 0.1;
   pose4.position.x += 0.1;
   waypoints.push_back(pose4);  // down and right (back to start) 


   moveit_msgs::RobotTrajectory trajectory_msg;
   group.setPlanningTime(30.0);
   double fraction = group.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory_msg, false);
  
   // Finally plan and execute the trajectory
   my_plan.trajectory_ = trajectory_msg;
   ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",fraction * 100.0);    
   sleep(5.0);
   group.execute(my_plan);

   ROS_INFO("Remove the object from the world");
   std::vector<std::string> object_ids;
   object_ids.push_back(collision_object.id);
   planning_scene_interface.removeCollisionObjects(object_ids);
   
   sleep(4.0);

   spinner.stop();
   return(0);
}
