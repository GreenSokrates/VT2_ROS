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
   // all.setPlannerId("RRTConnectkConfigDefault");

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
   pose2_ur3.orientation.y = -0.707;
   pose2_ur3.orientation.z = 0.0;
   all.setPoseTarget(pose2_ur3, ur3.getEndEffectorLink());     
   
   // Defining pose2_irb120
   geometry_msgs::Pose pose2_irb120;
   pose2_irb120.position.x = -0.2;
   pose2_irb120.position.y = 1.0-0.2;
   pose2_irb120.position.z = 0.5;
   pose2_irb120.orientation.w = 0.0;
   pose2_irb120.orientation.x = 0.0;
   pose2_irb120.orientation.y = 1.0;
   pose2_irb120.orientation.z = 0.0;
   all.setPoseTarget(pose2_irb120, irb120.getEndEffectorLink());
   
   // Defining pose2_tx90
   geometry_msgs::Pose pose2_tx90;
   pose2_tx90.position.x = -0.2;
   pose2_tx90.position.y = -1.0-0.2;
   pose2_tx90.position.z = 0.5;
   pose2_tx90.orientation.w = 0.707;
   pose2_tx90.orientation.x = 0.0;
   pose2_tx90.orientation.y = -0.707;
   pose2_tx90.orientation.z = 0.0;
   all.setPoseTarget(pose2_tx90, tx90.getEndEffectorLink());
   
   success = all.plan(my_plan);
   ROS_INFO("Planning of Plan 2: %s", success?"Succeded":"FAILED");
   // Execute movement if Planning succeded
   if(success) all.move();
   sleep(6.0);  
   
   // Defining pose3_ur3
   geometry_msgs::Pose pose3_ur3;
   pose3_ur3.position.x = -0.2;
   pose3_ur3.position.y = 0.25;
   pose3_ur3.position.z = 0.25;
   pose3_ur3.orientation.w = 0.0;
   pose3_ur3.orientation.x = 1.0;
   pose3_ur3.orientation.y = 0.0;
   pose3_ur3.orientation.z = 0.0;
   all.setPoseTarget(pose3_ur3, ur3.getEndEffectorLink());     
   
   // Defining pose3_irb120
   geometry_msgs::Pose pose3_irb120;
   pose3_irb120.position.x = -0.2;
   pose3_irb120.position.y = 1.0+0.25;
   pose3_irb120.position.z = 0.25;
   pose3_irb120.orientation.w = 0.707;
   pose3_irb120.orientation.x = 0.0;
   pose3_irb120.orientation.y = 0.707;
   pose3_irb120.orientation.z = 0.0;
   all.setPoseTarget(pose3_irb120, irb120.getEndEffectorLink());
   
   // Defining pose3_tx90
   geometry_msgs::Pose pose3_tx90;
   pose3_tx90.position.x = -0.2;
   pose3_tx90.position.y = -1.0+0.25;
   pose3_tx90.position.z = 0.25;
   pose3_tx90.orientation.w = 0.0;
   pose3_tx90.orientation.x = 1.0;
   pose3_tx90.orientation.y = 0.0;
   pose3_tx90.orientation.z = 0.0;
   all.setPoseTarget(pose3_tx90, tx90.getEndEffectorLink());   
   
   success = all.plan(my_plan);
   ROS_INFO("Planning of Plan 3: %s", success?"Succeded":"FAILED");
   // Execute movement if Planning succeded
   if(success) all.move();
   sleep(5.0);

   //Cartesian Movement from pose3
   // IRB120
   std::vector<geometry_msgs::Pose> waypoints_irb120;
   geometry_msgs::PoseStamped temp_irb120 = all.getCurrentPose(irb120.getEndEffectorLink());
   geometry_msgs::Pose pose4_irb120 = temp_irb120.pose;

   pose4_irb120.position.y -= 0.1;
   pose4_irb120.position.z -= 0.15;
   waypoints_irb120.push_back(pose4_irb120);

   pose4_irb120.position.x -= 0.1;
   waypoints_irb120.push_back(pose4_irb120);

   pose4_irb120.position.x += 0.15;
   pose4_irb120.position.y += 0.1;
   pose4_irb120.position.z += 0.1;
   waypoints_irb120.push_back(pose4_irb120);
   
   moveit_msgs::RobotTrajectory trajectory_msg;
   irb120.setPlanningTime(30.0); 
   double fraction = irb120.computeCartesianPath(waypoints_irb120,
                                            0.01,    //eef_step
                                            0.0,    // jump_threshold
                                            trajectory_msg, false);

   my_plan.trajectory_ = trajectory_msg;
   ROS_INFO("Visualizing Cartesian Path IRB120 (%2f%% acheived)", fraction *100.0);
   sleep(5.0);
   irb120.execute(my_plan);

   // UR3
   std::vector<geometry_msgs::Pose> waypoints_ur3;
   geometry_msgs::PoseStamped temp_ur3 = all.getCurrentPose(ur3.getEndEffectorLink());
   geometry_msgs::Pose pose4_ur3 = temp_ur3.pose;

   pose4_ur3.position.y -= 0.1;
   pose4_ur3.position.z -= 0.15;
   waypoints_ur3.push_back(pose4_ur3);

   pose4_ur3.position.x -= 0.1;
   waypoints_ur3.push_back(pose4_ur3);

   pose4_ur3.position.x += 0.15;
   pose4_ur3.position.y += 0.1;
   pose4_ur3.position.z += 0.1;
   waypoints_ur3.push_back(pose4_ur3);
   
   irb120.setPlanningTime(30.0); 
   fraction = ur3.computeCartesianPath(waypoints_ur3,
                                            0.01,    //eef_step
                                            0.0,    // jump_threshold
                                            trajectory_msg, false);

   my_plan.trajectory_ = trajectory_msg;
   ROS_INFO("Visualizing Cartesian Path UR3 (%2f%% acheived)", fraction *100.0);
   sleep(5.0);
   ur3.execute(my_plan);
   
   // TX90
   std::vector<geometry_msgs::Pose> waypoints_tx90;
   geometry_msgs::PoseStamped temp_tx90 = all.getCurrentPose(tx90.getEndEffectorLink());
   geometry_msgs::Pose pose4_tx90 = temp_tx90.pose;

   pose4_tx90.position.y -= 0.2;
   pose4_tx90.position.z -= 0.3;
   waypoints_tx90.push_back(pose4_tx90);

   pose4_tx90.position.x -= 0.3;
   waypoints_tx90.push_back(pose4_tx90);

   pose4_tx90.position.x += 0.2;
   pose4_tx90.position.y += 0.3;
   pose4_tx90.position.z += 0.3;
   waypoints_tx90.push_back(pose4_tx90);   
   
   tx90.setPlanningTime(30.0); 
   fraction = tx90.computeCartesianPath(waypoints_tx90,
                                            0.01,    //eef_step
                                            0.0,    // jump_threshold
                                            trajectory_msg, false);

   my_plan.trajectory_ = trajectory_msg;
   ROS_INFO("Visualizing Cartesian Path TX90 (%2f%% acheived)", fraction *100.0);
   sleep(5.0);
   tx90.execute(my_plan);

   sleep(4.0);  

   spinner.stop();
   return(0);
}
