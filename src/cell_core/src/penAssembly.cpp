#include <cell_core/penAssembly.h>
using namespace std;

geometry_msgs::Pose montage;
geometry_msgs::Pose montageRHull;
geometry_msgs::Pose pickTool;
geometry_msgs::Pose pickBase;
geometry_msgs::Pose pickFHull;
geometry_msgs::Pose pickRHull;
geometry_msgs::Pose pickInk;
geometry_msgs::Pose pickSpring;
geometry_msgs::Pose pickArr;

void MoveLinear(double x, double y, double z, moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    std::vector<geometry_msgs::Pose> waypoints_tool;
    geometry_msgs::PoseStamped temp_montage = group->getCurrentPose(group->getEndEffectorLink());
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
    group->setPlanningTime(30.0);
    double fraction = group->computeCartesianPath(waypoints_tool,
                                                  0.01, //eef_step
                                                  0.0,  // jump_threshold
                                                  trajectory_msg, false);
    plan.trajectory_ = trajectory_msg;
    ROS_INFO("Visualizing Cartesian Path (%2f%% acheived)", fraction * 100.0);
    sleep(5.0);
    group->execute(plan);
    group->setPlanningTime(10.0);
}

void MoveToPose(geometry_msgs::Pose &position, moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    group->setPoseTarget(position);
    bool success = group->plan(plan);
    if (success)
        group->move();
    return;
}

void initPoses(double offset)
{
    pickBase.position.x = 0.148 + offset;
    pickBase.position.y = 0.260;
    pickBase.position.z = 0.100;
    pickBase.orientation.w = 0.0;
    pickBase.orientation.x = 1.0;
    pickBase.orientation.y = 0.0;
    pickBase.orientation.z = 0.0;

    montage.position.x = -0.040 + 0.05;
    montage.position.y = 0.603;
    montage.position.z = 0.114 + 0.05;
    montage.orientation.w = -0.271;
    montage.orientation.x = 0.271;
    montage.orientation.y = 0.653;
    montage.orientation.z = 0.653;

    montageRHull.position.x = -0.166;
    montageRHull.position.y = 0.594;
    montageRHull.position.z = 0.188;
    montageRHull.orientation.w = 0.665;
    montageRHull.orientation.x = -0.665;
    montageRHull.orientation.y = -0.238;
    montageRHull.orientation.z = 0.238;

    pickTool.position.x = -0.22;
    pickTool.position.y = 0.507;
    pickTool.position.z = 0.083;
    pickTool.orientation.w = -0.271;
    pickTool.orientation.x = 0.653;
    pickTool.orientation.y = 0.653;
    pickTool.orientation.z = -0.271;

    pickFHull = pickBase;
    pickFHull.position.x += 0.0270;
    pickFHull.position.y += 0.0275;
    pickFHull.position.z += 0.0000;

    pickRHull = pickBase;
    pickRHull.position.x += 0.047;
    pickRHull.position.y += 0.129;
    pickRHull.position.z += 0.000;
    pickRHull.orientation.w = 0.707;
    pickRHull.orientation.x = 0.0;
    pickRHull.orientation.y = 0.0;
    pickRHull.orientation.z = 0.707;

    pickInk = pickBase;
    pickInk.position.x += 0.0690;
    pickInk.position.y += 0.0725;
    pickInk.position.z += 0.0000;

    pickSpring = pickBase;
    pickSpring.position.x += 0.0203;
    pickSpring.position.y += 0.167;
    pickSpring.position.z += 0.000;
    pickSpring.orientation.w = 0.707;
    pickSpring.orientation.x = 0.0;
    pickSpring.orientation.y = 0.0;
    pickSpring.orientation.z = 0.707;

    pickArr = pickBase;
    pickArr.position.x += 0.0705;
    pickArr.position.y += 0.172;
    pickArr.position.z += 0.000;
    pickArr.orientation.w = 0.707;
    pickArr.orientation.x = 0.0;
    pickArr.orientation.y = 0.0;
    pickArr.orientation.z = 0.707;
}

bool montageCallback(cell_core::montage_service::Request &req, cell_core::montage_service::Response &res)
{
    if (req.Ausgabestelle == 1 && req.Ausgabestelle == 2)
    {
        initPoses(req.xOffset);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        
        MoveToPose(pickFHull, my_plan);
        MoveLinear(0.0, 0.0, -0.07, my_plan);
        MoveToPose(montage, my_plan);
        MoveLinear(-0.05, 0.0, -0.05, my_plan);
        MoveToPose(pickTool, my_plan);
        ROS_INFO("pickTool");
        MoveToPose(montage, my_plan);
        ROS_INFO("montage");
        MoveToPose(pickSpring, my_plan);
        MoveLinear(0.0, 0.0, -0.07, my_plan);
        ROS_INFO("pickSpring");
        MoveToPose(montage, my_plan);
        ROS_INFO("montage");
        MoveToPose(pickInk, my_plan);
        MoveLinear(0.0, 0.0, -0.07, my_plan);
        ROS_INFO("pickInk");
        MoveToPose(montage, my_plan);
        ROS_INFO("montage");
        MoveToPose(pickArr, my_plan);
        MoveLinear(0.0, 0.0, -0.07, my_plan);
        ROS_INFO("pickArr");
        MoveToPose(montage, my_plan);
        ROS_INFO("montage");
        // TODO: circMove
        MoveToPose(pickRHull, my_plan);
        MoveLinear(0.0, 0.0, -0.07, my_plan);
        ROS_INFO("pickRHull");
        MoveToPose(montageRHull, my_plan);
        ROS_INFO("montageRHull");
        /*if(req.Ausgabestelle == 1){
        MoveToPose(Ausgabe1, my_plan);
        // Greiffer öffnen
    }
    else if (req.Ausgabestelle == 2){
        MoveToPose(Ausgabe2, my_plan);
    }
    MoveToPose(Home, my_plan);*/
        return true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "penAssembly");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);

    spinner.start();

    group.reset(new moveit::planning_interface::MoveGroupInterface("gripper_eef"));
    group->setPoseReferenceFrame("/base_link");
    group->setPlannerId("RRTConnectkConfigDefault");
    group->setPlanningTime(2);

    //my_plan.reset(new moveit::planning_interface::MoveGroupInterface::Plan);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    collisionObjectAdder coAdder;
    coAdder.addCell();
    sleep(10);

    /*
    planning_scene_interface = new (moveit::planning_interface::PlanningSceneInterface);
    group = new (moveit::planning_interface::MoveGroupInterface group("gripper_eef"));
    my_plan = new (moveit::planning_interface::MoveGroupInterface::Plan);
*/
    ros::ServiceServer service = nh.advertiseService("montage_service", montageCallback);

    ros::waitForShutdown();
}