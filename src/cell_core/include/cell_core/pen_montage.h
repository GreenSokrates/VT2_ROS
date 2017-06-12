#ifndef PENMONTAGE_H_
#define PENMONTAGE_H_

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <cell_core/collisionObject.h>
#include "std_msgs/String.h"

// Defining Positions and Offsets
static geometry_msgs::Pose montage;
static geometry_msgs::Pose montageRHull;
static geometry_msgs::Pose pickTool;
static geometry_msgs::Pose pickBase;
static geometry_msgs::Pose pickFHull;
static geometry_msgs::Pose pickRHull;
static geometry_msgs::Pose pickInk;
static geometry_msgs::Pose pickSpring;
static geometry_msgs::Pose pickArr;

#endif