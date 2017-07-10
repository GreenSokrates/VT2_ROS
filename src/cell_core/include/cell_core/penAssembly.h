#ifndef PENASSEMBLY_H_
#define PENASSEMBLY_H_

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <cell_core/collisionObject.h>
#include "std_msgs/String.h"

geometry_msgs::Pose montage;
geometry_msgs::Pose montageRHull;
geometry_msgs::Pose pickTool;
geometry_msgs::Pose pickBase;
geometry_msgs::Pose pickFHull;
geometry_msgs::Pose pickRHull;
geometry_msgs::Pose pickInk;
geometry_msgs::Pose pickSpring;
geometry_msgs::Pose pickArr;

#endif