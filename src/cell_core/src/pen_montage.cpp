#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometric_shapes/shape_operations.h>

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
   montage.position.x = 0.0045;
   montage.position.y = 0.420;
   montage.position.z = 0.130;
   montage.orientation.w = 0.653;
   montage.orientation.x = 0.271;
   montage.orientation.y = 0.653;
   montage.orientation.z = 0.271;
   
   geometry_msgs::Pose pickBase;
   pickBase.position.x = 0.148;
   pickBase.position.y = 0.260;
   pickBase.position.z = 0.050;
   pickBase.orientation.w = 0.707;
   pickBase.orientation.x = 0.0;
   pickBase.orientation.y = 0.707;
   pickBase.orientation.z = 0.0;

   // Generating Collision object from Mesh
   moveit_msgs::CollisionObject co;
   co.id = "cell";
   shapes::Mesh* m = shapes::createMeshFromResource("package://cell_support/mesh/model_v3_meter.stl");
   ROS_INFO("Mesh Loaded");

   shape_msgs::Mesh mesh;
   shapes::ShapeMsg mesh_msg;
   shapes::constructMsgFromShape(m, mesh_msg);
   mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

   co.meshes.resize(1);
   co.mesh_poses.resize(1);
   co.meshes[0] = mesh;
   co.header.frame_id = "wall";
   co.mesh_poses[0].position.x = -1.175;
   co.mesh_poses[0].position.y = 0.222;
   co.mesh_poses[0].position.z = -0.010;
   co.mesh_poses[0].orientation.w = 0.707;
   co.mesh_poses[0].orientation.x = 0.0;
   co.mesh_poses[0].orientation.y = 0.0;
   co.mesh_poses[0].orientation.z = 0.707;

   co.meshes.push_back(mesh);
   co.mesh_poses.push_back(co.mesh_poses[0]);
   co.operation = co.ADD;
   std::vector<moveit_msgs::CollisionObject> vec;
   vec.push_back(co);
   ROS_INFO("Wall added into the world");
   planning_scene_interface.addCollisionObjects(vec);
   sleep(10.0);
   ROS_INFO("End of co");
   
      
   
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


