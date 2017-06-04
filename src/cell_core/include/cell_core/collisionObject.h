#ifndef COLLISIONOBJECTADDER_H_
#define COLLISIONOBJECTADDER_H_

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/move_group_interface/move_group_interface.h>

class collisionObjectAdder
{
protected:
  ros::NodeHandle nh;
  ros::Publisher add_collision_object_pub;
  //ros::Publisher planning_scene_diff_pub;
public:
  collisionObjectAdder();
  void addCollisionCell(moveit::planning_interface::PlanningSceneInterface &psi, moveit::planning_interface::MoveGroupInterface &grouper);
  void addCollisionObject();
  //int collisionObjectAdder::main(int argc, char **argv);
};

#endif /*COLLISIONOBJECTADDER_H_ */