#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <cell_core/collisionObject.h>

using namespace ros;
using namespace moveit;
static bool setup = 0;

void moveToPoint(geometry_msgs::Pose &position, planning_interface::MoveGroupInterface::Plan &planer, planning_interface::MoveGroupInterface &grouper)
{
    grouper.setPoseTarget(position);
    bool success = grouper.plan(planer);
    if (success)
        grouper.move();
    return;
}

void moveLinear(double x, double y, double z, planning_interface::MoveGroupInterface::Plan &planer, planning_interface::MoveGroupInterface &group)
{
    std::vector<geometry_msgs::Pose> waypoints_tool;
    geometry_msgs::PoseStamped temp_montage = group.getCurrentPose(group.getEndEffectorLink());
    geometry_msgs::Pose test_pose = temp_montage.pose;

    test_pose.position.x += x;
    test_pose.position.y += y;
    test_pose.position.z += z;
    waypoints_tool.push_back(test_pose);
    test_pose.position.x -= x;
    test_pose.position.y -= y;
    test_pose.position.z -= z;
    waypoints_tool.push_back(test_pose);

    moveit_msgs::RobotTrajectory trajectory_msg;
    group.setPlanningTime(30.0);
    double fraction = group.computeCartesianPath(waypoints_tool,
                                                 0.01, //eef_step
                                                 0.0,  // jump_threshold
                                                 trajectory_msg, false);
    planer.trajectory_ = trajectory_msg;
    ROS_INFO("Visualizing Cartesian Path (%2f%% acheived)", fraction * 100.0);
    sleep(5.0);
    group.execute(planer);
    group.setPlanningTime(10.0);
}

int main(int argc, char **argv)
{
#include <moveit/planning_scene_interface/planning_scene_interface.h>
    init(argc, argv, "pen_montage");
    NodeHandle n;

    // Defining Positions and Offsets
    geometry_msgs::Pose montage;
    montage.position.x = -0.040 + 0.05;
    montage.position.y = 0.603;
    montage.position.z = 0.114 + 0.05;
    montage.orientation.w = -0.271;
    montage.orientation.x = 0.271;
    montage.orientation.y = 0.653;
    montage.orientation.z = 0.653;

    geometry_msgs::Pose montageRHull;
    montageRHull.position.x = -0.166;
    montageRHull.position.y = 0.590;
    montageRHull.position.z = 0.188;
    montageRHull.orientation.w = 0.183;
    montageRHull.orientation.x = 0.008;
    montageRHull.orientation.y = -0.677;
    montageRHull.orientation.z = -0.713;

    geometry_msgs::Pose pickTool;
    pickTool.position.x = -0.22;
    pickTool.position.y = 0.507;
    pickTool.position.z = 0.083;
    pickTool.orientation.w = -0.271;
    pickTool.orientation.x = 0.653;
    pickTool.orientation.y = 0.653;
    pickTool.orientation.z = -0.271;

    geometry_msgs::Pose pickBase;
    pickBase.position.x = 0.148;
    pickBase.position.y = 0.260;
    pickBase.position.z = 0.100;
    pickBase.orientation.w = 0.0;
    pickBase.orientation.x = 1.0;
    pickBase.orientation.y = 0.0;
    pickBase.orientation.z = 0.0;

    geometry_msgs::Pose pickFHull = pickBase;
    pickFHull.position.x += 0.0270;
    pickFHull.position.y += 0.0275;
    pickFHull.position.z += 0.0000;

    geometry_msgs::Pose pickRHull = pickBase;
    pickRHull.position.x += 0.047;
    pickRHull.position.y += 0.129;
    pickRHull.position.z += 0.000;
    pickRHull.orientation.w = 0.0;
    pickRHull.orientation.x = 0.707;
    pickRHull.orientation.y = 0.707;
    pickRHull.orientation.z = 0.0;

    geometry_msgs::Pose pickInk = pickBase;
    pickInk.position.x += 0.0690;
    pickInk.position.y += 0.0725;
    pickInk.position.z += 0.0000;

    geometry_msgs::Pose pickSpring = pickBase;
    pickSpring.position.x += 0.0203;
    pickSpring.position.y += 0.167;
    pickSpring.position.z += 0.000;
    pickSpring.orientation.w = 0.0;
    pickSpring.orientation.x = 0.707;
    pickSpring.orientation.y = 0.707;
    pickSpring.orientation.z = 0.0;

    geometry_msgs::Pose pickArr = pickBase;
    pickArr.position.x += 0.0705;
    pickArr.position.y += 0.172;
    pickArr.position.z += 0.000;
    pickArr.orientation.w = 0.0;
    pickArr.orientation.x = 0.707;
    pickArr.orientation.y = 0.707;
    pickArr.orientation.z = 0.0;

    // Setup of MoveGroupInterface and PlanningSceneInterface
    planning_interface::MoveGroupInterface group("gripper_tool");
    planning_interface::PlanningSceneInterface planning_scene_interface;
    group.setPlannerId("LBKPIECE");

    // CALL COLLISION
    /*if (!setup)
    {
        collisionObjectAdder coAdder;
        coAdder.addCell(planning_scene_interface, group);
        sleep(10.0);
        setup = 1;
    } */

    // Construction of planner
    planning_interface::MoveGroupInterface::Plan my_plan;

    /**********************
    ***Start of Montage ***
    ***********************/
    AsyncSpinner spinner(1);
    spinner.start();

    moveToPoint(pickFHull, my_plan, group);
    moveLinear(0.0, 0.0, -0.07, my_plan, group);
    moveToPoint(montage, my_plan, group);
    moveLinear(-0.05, 0.0, -0.05, my_plan, group);
    moveToPoint(pickTool, my_plan, group);
    ROS_INFO("pickTool");
    moveToPoint(montage, my_plan, group);
    ROS_INFO("montage");
    moveToPoint(pickSpring, my_plan, group);
    moveLinear(0.0, 0.0, -0.07, my_plan, group);
    ROS_INFO("pickSpring");
    moveToPoint(montage, my_plan, group);
    ROS_INFO("montage");
    moveToPoint(pickInk, my_plan, group);
    moveLinear(0.0, 0.0, -0.07, my_plan, group);
    ROS_INFO("pickInk");
    moveToPoint(montage, my_plan, group);
    ROS_INFO("montage");
    moveToPoint(pickArr, my_plan, group);
    moveLinear(0.0, 0.0, -0.07, my_plan, group);
    ROS_INFO("pickArr");
    moveToPoint(montage, my_plan, group);
    ROS_INFO("montage");
    // TODO: circMove
    moveToPoint(pickRHull, my_plan, group);
    moveLinear(0.0, 0.0, -0.07, my_plan, group);
    ROS_INFO("pickRHull");
    moveToPoint(montageRHull, my_plan, group);
    ROS_INFO("montageRHull");

    // spinner.stop();
    return (0);
}