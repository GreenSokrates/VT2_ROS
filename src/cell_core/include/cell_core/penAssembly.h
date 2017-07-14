#ifndef PENASSEMBLY_H_
#define PENASSEMBLY_H_

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <cell_core/collisionObject.h>
#include "std_msgs/String.h"
#include <ros/service.h>
#include <cell_core/montage_service.h>

boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> group;
//boost::shared_ptr<moveit::planning_interface::MoveGroupInterface::Plan> my_plan;

#endif /* PENASSEMBLY_H_ */