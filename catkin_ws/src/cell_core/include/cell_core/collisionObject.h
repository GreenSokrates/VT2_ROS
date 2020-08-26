#ifndef COLLISION_OBJECT_H_
#define COLLISION_OBJECT_H_

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/move_group_interface/move_group_interface.h>

class collisionObject
{
protected:
  ros::NodeHandle nh;
  ros::Publisher add_collision_object_pub;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

public:
  collisionObject();
  virtual ~collisionObject(){};
  void addCell(boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> &group);
  void addBody(boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> &group, int penPart);
  void removeBody(boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> &group, int penPart);
  void detatchBody(boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> &group, int penPart);
};

#endif /*COLLISION_OBJECT_H_ */