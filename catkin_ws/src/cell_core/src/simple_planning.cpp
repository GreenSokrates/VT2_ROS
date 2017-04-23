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

    // Initialising Pose Named 'pose1'
    geometry_msgs::Pose pose1;
    pose1.position.x = 0.0;
    pose1.position.y = 0.0;
    pose1.position.z = 0.8;
    pose1.orientation.w = 0.707;
    pose1.orientation.x = 0.0;
    pose1.orientation.y = 0.0;
    pose1.orientation.z = 0.707;


    group.setPoseTarget(pose1);
    group.setPlanningTime(20.0);
    // group.setPlannerId("RRTConnectkConfigDefault");

    // Construction of planner
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = group.plan(my_plan);
    ROS_INFO("Planning of Plan 1: %s", success?"Succeded":"FAILED");
    sleep(2.0);
    // Execute movement if Planning succeded
    if(success) group.move();



    spinner.stop();
    return(0);
}