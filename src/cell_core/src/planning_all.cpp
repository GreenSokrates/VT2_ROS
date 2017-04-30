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
   planning_interface::MoveGroupInterface ur3("ur3");       
   planning_interface::MoveGroupInterface irb120("irb120");
   planning_interface::MoveGroupInterface tx90("tx90");
   planning_interface::MoveGroupInterface all("all_sub");

   
   planning_interface::PlanningSceneInterface planning_scene_interface;
   Publisher display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
   moveit_msgs::DisplayTrajectory display_trajectory;

   // Getting basic Infos from Robot
   ROS_INFO("Reference frame: %s", all.getPlanningFrame().c_str());
   ROS_INFO("Reference frame: %s", all.getEndEffectorLink().c_str());


   // Defining pose1_ur3
   geometry_msgs::Pose pose1_ur3;
   pose1_ur3.position.x = 0.15;
   pose1_ur3.position.y = 0.15;
   pose1_ur3.position.z = 0.65;
   pose1_ur3.orientation.w = 0.707;
   pose1_ur3.orientation.x = 0.0;
   pose1_ur3.orientation.y = 0.0;
   pose1_ur3.orientation.z = 0.707;
   all.setPoseTarget(pose1_ur3, ur3.getEndEffectorLink());

   // Defining pose1_irb120
   geometry_msgs::Pose pose1_irb120;
   pose1_irb120.position.x = 0.15;
   pose1_irb120.position.y = 1.0+0.15;
   pose1_irb120.position.z = 0.65;
   pose1_irb120.orientation.w = 0.707;
   pose1_irb120.orientation.x = 0.0;
   pose1_irb120.orientation.y = -0.707;
   pose1_irb120.orientation.z = 0.0;
   all.setPoseTarget(pose1_irb120, irb120.getEndEffectorLink());

   // Defining pose1_tx90
   geometry_msgs::Pose pose1_tx90;
   pose1_tx90.position.x = 0.3;
   pose1_tx90.position.y = -1.0+0.3;
   pose1_tx90.position.z = 0.65;
   pose1_tx90.orientation.w = 0.707;
   pose1_tx90.orientation.x = 0.0;
   pose1_tx90.orientation.y = 0.0;
   pose1_tx90.orientation.z = 0.707;
   all.setPoseTarget(pose1_tx90, tx90.getEndEffectorLink());
   
   // Construction of planner
   moveit::planning_interface::MoveGroupInterface::Plan my_plan;
   all.setPlanningTime(20);

   bool success = all.plan(my_plan);
   ROS_INFO("Planning of Plan 1: %s", success?"Succeded":"FAILED");
   sleep(2.0);
   // Execute movement if Planning succeded
   if(success) all.move();
   sleep(5.0);
   
   
   // Defining pose2_ur3
   geometry_msgs::Pose pose2_ur3;
   pose2_ur3.position.x = -0.2;
   pose2_ur3.position.y = -0.2;
   pose2_ur3.position.z = 0.5;
   pose2_ur3.orientation.w = 0.707;
   pose2_ur3.orientation.x = 0.0;
   pose2_ur3.orientation.y = 0.707;
   pose2_ur3.orientation.z = 0.0;
   all.setPoseTarget(pose2_ur3, ur3.getEndEffectorLink());     
   
   // Defining pose2_irb120
   geometry_msgs::Pose pose2_irb120;
   pose2_irb120.position.x = -0.2;
   pose2_irb120.position.y = 1.0-0.2;
   pose2_irb120.position.z = 0.5;
   pose2_irb120.orientation.w = 0.707;
   pose2_irb120.orientation.x = 0.0;
   pose2_irb120.orientation.y = 0.0;
   pose2_irb120.orientation.z = 0.-707;
   all.setPoseTarget(pose2_irb120, irb120.getEndEffectorLink());
   
   // Defining pose2_tx90
   geometry_msgs::Pose pose2_tx90;
   pose2_tx90.position.x = -0.2;
   pose2_tx90.position.y = -1.0-0.2;
   pose2_tx90.position.z = 0.5;
   pose2_tx90.orientation.w = 0.707;
   pose2_tx90.orientation.x = 0.0;
   pose2_tx90.orientation.y = 0.707;
   pose2_tx90.orientation.z = 0.0;
   all.setPoseTarget(pose2_tx90, tx90.getEndEffectorLink());
   
   success = all.plan(my_plan);
   ROS_INFO("Planning of Plan 2: %s", success?"Succeded":"FAILED");
   sleep(2.0);
   // Execute movement if Planning succeded
   if(success) all.move();
   sleep(5.0);  
   
   // Defining pose3_ur3
   geometry_msgs::Pose pose3_ur3;
   pose3_ur3.position.x = -0.2;
   pose3_ur3.position.y = 0.2;
   pose3_ur3.position.z = 0.25;
   pose3_ur3.orientation.w = 0.0;
   pose3_ur3.orientation.x = 1.0;
   pose3_ur3.orientation.y = 0.0;
   pose3_ur3.orientation.z = 0.0;
   all.setPoseTarget(pose3_ur3, ur3.getEndEffectorLink());     
   
   // Defining pose3_irb120
   geometry_msgs::Pose pose3_irb120;
   pose3_irb120.position.x = -0.2;
   pose3_irb120.position.y = 1.0+0.2;
   pose3_irb120.position.z = 0.25;
   pose3_irb120.orientation.w = 0.707;
   pose3_irb120.orientation.x = 0.0;
   pose3_irb120.orientation.y = 0.707;
   pose3_irb120.orientation.z = 0.0;
   all.setPoseTarget(pose3_irb120, irb120.getEndEffectorLink());
   
   // Defining pose3_tx90
   geometry_msgs::Pose pose3_tx90;
   pose3_tx90.position.x = -0.2;
   pose3_tx90.position.y = -1.0+0.2;
   pose3_tx90.position.z = 0.25;
   pose3_tx90.orientation.w = 0.0;
   pose3_tx90.orientation.x = 1.0;
   pose3_tx90.orientation.y = 0.0;
   pose3_tx90.orientation.z = 0.0;
   all.setPoseTarget(pose3_tx90, tx90.getEndEffectorLink());
   
   
   success = all.plan(my_plan);
   ROS_INFO("Planning of Plan 3: %s", success?"Succeded":"FAILED");
   sleep(2.0);
   // Execute movement if Planning succeded
   if(success) all.move();
   sleep(5.0);
  

   spinner.stop();
   return(0);
}
