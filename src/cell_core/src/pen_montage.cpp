#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometric_shapes/shape_operations.h>

using namespace ros;
using namespace moveit;

int main(int argc, char **argv)
{
    init(argc, argv, "movement");
    NodeHandle n;

    AsyncSpinner spinner(1);
    spinner.start();

    // Setup of MoveGroupInterface and PlanningSceneInterface
    planning_interface::MoveGroupInterface group("manipulator");
    planning_interface::PlanningSceneInterface planning_scene_interface;
    Publisher display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    group.setPlannerId("SBLkConfigDefault");

    // Getting basic Infos from Robot
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    // Defining Positions and Offsets
    geometry_msgs::Pose montage;
    montage.position.x = -0.040;
    montage.position.y = 0.603;
    montage.position.z = 0.114;
    montage.orientation.w = 0.653;
    montage.orientation.x = -0.653;
    montage.orientation.y = 0.271;
    montage.orientation.z = 0.271;

    geometry_msgs::Pose pickTool;
    pickTool.position.x = -0.187;
    pickTool.position.y = 0.507;
    pickTool.position.z = 0.083;
    pickTool.orientation.w = 0.000;
    pickTool.orientation.x = 0.924;
    pickTool.orientation.y = -0.000;
    pickTool.orientation.z = -0.383;

    geometry_msgs::Pose pickBase;
    pickBase.position.x = 0.148;
    pickBase.position.y = 0.200;
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
    pickRHull.orientation.w = 0.707;
    pickRHull.orientation.x = 0.0;
    pickRHull.orientation.y = 0.0;
    pickRHull.orientation.z = 0.707;

    geometry_msgs::Pose pickInk = pickBase;
    pickInk.position.x += 0.0690;
    pickInk.position.y += 0.0725;
    pickInk.position.z += 0.0000;

    geometry_msgs::Pose pickSpring = pickBase;
    pickSpring.position.x += 0.0203;
    pickSpring.position.y += 0.167;
    pickSpring.position.z += 0.000;

    geometry_msgs::Pose pickArr = pickBase;
    pickArr.position.x += 0.0705;
    pickArr.position.y += 0.172;
    pickArr.position.z += 0.000;

    // Generating Collision object from Mesh
    Eigen::Vector3d scaling_vector(0.001, 0.001, 0.001); // Scaling Vector
    moveit_msgs::CollisionObject co;
    co.id = "cell";
    shapes::Mesh *m = shapes::createMeshFromResource("package://cell_support/meshes/Mittelteil_final.stl", scaling_vector);
    ROS_INFO("Mesh Loaded");

    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    co.meshes.resize(1);
    co.mesh_poses.resize(1);
    co.meshes[0] = mesh;
    co.header.frame_id = "Cell";
    co.mesh_poses[0].position.x = -1.075;
    co.mesh_poses[0].position.y = 0.023;
    co.mesh_poses[0].position.z = -0.021;
    co.mesh_poses[0].orientation.w = 0.707;
    co.mesh_poses[0].orientation.x = 0.0;
    co.mesh_poses[0].orientation.y = 0.0;
    co.mesh_poses[0].orientation.z = 0.707;

    co.meshes.push_back(mesh);
    co.mesh_poses.push_back(co.mesh_poses[0]);
    co.operation = co.ADD;
    std::vector<moveit_msgs::CollisionObject> vec;
    vec.push_back(co);
    ROS_INFO("Cell added into the world");
    planning_scene_interface.addCollisionObjects(vec);
    sleep(10.0);

    // Construction of planner
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    group.setPoseTarget(pickFHull);
    bool success = group.plan(my_plan);
    ROS_INFO("Planning to pick Front Hull: %s", success ? "Succeded" : "FAILED");
    if (success)
        group.move();

    group.setPoseTarget(montage);
    success = group.plan(my_plan);
    ROS_INFO("Planning to mount Front Hull: %s", success ? "Succeded" : "FAILED");
    //sleep(2.0);
    if (success)
        group.move();
    //sleep(5.0);

    /*  group.setPoseTarget(pickTool);
    success = group.plan(my_plan);
    ROS_INFO("Planning to pick: Tool: %s", success ? "Succeded" : "FAILED");
    if (success)
        group.move();

    group.setPoseTarget(montage);
    success = group.plan(my_plan);
    ROS_INFO("Planning to mount Front Hull: %s", success ? "Succeded" : "FAILED");
    if (success)
        group.move();

    group.setPoseTarget(pickTool);
    success = group.plan(my_plan);
    ROS_INFO("Planning to pick: Tool: %s", success ? "Succeded" : "FAILED");
    if (success)
        group.move();

    group.setPoseTarget(pickSpring);
    success = group.plan(my_plan);
    ROS_INFO("Planning to pick: Spring: %s", success ? "Succeded" : "FAILED");
    //sleep(2.0);
    // Execute movement if Planning succeded
    if (success)
        group.move();
    //sleep(5.0);

    group.setPoseTarget(montage);
    success = group.plan(my_plan);
    ROS_INFO("Planning to mount: Spring: %s", success ? "Succeded" : "FAILED");
    //sleep(2.0);
    // Execute movement if Planning succeded
    if (success)
        group.move();
    //sleep(5.0);

    group.setPoseTarget(pickInk);
    success = group.plan(my_plan);
    ROS_INFO("Planning to pick Ink: %s", success ? "Succeded" : "FAILED");
    //sleep(2.0);
    // Execute movement if Planning succeded
    if (success)
        group.move();
    sleep(5.0);

    group.setPoseTarget(montage);
    success = group.plan(my_plan);
    ROS_INFO("Planning to mount Ink: %s", success ? "Succeded" : "FAILED");
    //sleep(2.0);
    // Execute movement if Planning succeded
    if (success)
        group.move();
    //sleep(5.0);

    group.setPoseTarget(pickArr);
    success = group.plan(my_plan);
    ROS_INFO("Planning to pick Arr: %s", success ? "Succeded" : "FAILED");
    //sleep(2.0);
    // Execute movement if Planning succeded
    if (success)
        group.move();
    //sleep(5.0);

    group.setPoseTarget(montage);
    success = group.plan(my_plan);
    ROS_INFO("Planning to mount Arr: %s", success ? "Succeded" : "FAILED");
    //sleep(2.0);
    // Execute movement if Planning succeded
    if (success)
        group.move();
    //sleep(5.0);

    group.setPoseTarget(pickRHull);
    success = group.plan(my_plan);
    ROS_INFO("Planning to pick Rear Hull: %s", success ? "Succeded" : "FAILED");
    if (success)
        group.move();

    group.setPoseTarget(montage);
    success = group.plan(my_plan);
    ROS_INFO("Planning to mount Rear Hull: %s", success ? "Succeded" : "FAILED");
    if (success)
        group.move(); */ 1

        spinner.stop();
    return (0);
}