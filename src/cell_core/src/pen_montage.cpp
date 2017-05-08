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
   geometry_msgs::Pose montage;
   montage.position.x = 0.260;
   montage.position.y = 0.0045;
   montage.position.z = 0.130;
   montage.orientation.w = 1.0;
   montage.orientation.x = 0.0;
   montage.orientation.y = 0.0;
   montage.orientation.z = 0.0;
   
   geometry_msgs::Pose pickBase;
   pickBase.position.x = 0.260;
   pickBase.position.y = 0.148;
   pickBase.position.z = 0.050;
   pickBase.orientation.w = 1.0;
   pickBase.orientation.x = 0.0;
   pickBase.orientation.y = 0.0;
   pickBase.orientation.z = 0.0;

   
   
   group.setPoseTarget(montage);

   // Construction of planner
   moveit::planning_interface::MoveGroupInterface::Plan my_plan;

   bool success = group.plan(my_plan);
   ROS_INFO("Planning of Plan 1: %s", success?"Succeded":"FAILED");
   sleep(2.0);
   // Execute movement if Planning succeded
   if(success) group.move();
   sleep(5.0);

   group.setPoseTarget(pickBase);
   success = group.plan(my_plan);
   ROS_INFO("Planning of Plan 1: %s", success?"Succeded":"FAILED");
   sleep(2.0);
   // Execute movement if Planning succeded
   if(success) group.move();
   sleep(5.0);

   spinner.stop();
   return(0);
}


