#ifndef PENASSEMBLY_H_
#define PENASSEMBLY_H_

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <cell_core/collisionObject.h>

using namespace ros;

class penAssembly
{
  public:
	penAssembly();			// Constructor
	virtual ~penAssembly(); // Destructor

	void MoveToPose(geometry_msgs::Pose &,
					moveit::planning_interface::MoveGroupInterface::Plan &,
					moveit::planning_interface::MoveGroupInterface &); // Moves arm to a Point

	void MoveLinear(double x,
					double y,
					double z,
					moveit::planning_interface::MoveGroupInterface::Plan &planer,
					moveit::planning_interface::MoveGroupInterface &group);

	void AssemblePen(); // Starts the Assembly of one Pen

	static geometry_msgs::Pose montage;
	static geometry_msgs::Pose montageRHull;
	static geometry_msgs::Pose pickTool;
	static geometry_msgs::Pose pickBase;
	static geometry_msgs::Pose pickFHull;
	static geometry_msgs::Pose pickRHull;
	static geometry_msgs::Pose pickInk;
	static geometry_msgs::Pose pickSpring;
	static geometry_msgs::Pose pickArr;

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit::planning_interface::MoveGroupInterface group("gripper_eef");
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	/*class planning_scene_interface;
	class moveit::planning_interface::MoveGroupInterface group("gripper_eef");
	struct my_plan;*/
};

#endif