#include <cell_core/penAssembly.h>
using namespace std;

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
                                                  0.001, //eef_step
                                                  0.0,   // jump_threshold
                                                  trajectory_msg, true);
    plan.trajectory_ = trajectory_msg;
    ROS_INFO("Visualizing Cartesian Path (%2f%% acheived)", fraction * 100.0);
    sleep(5.0);
    group->execute(plan);
    group->setPlanningTime(10.0);
}

bool MoveToPose(geometry_msgs::Pose &position, moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    group->setPoseTarget(position);
    bool success = group->plan(plan);
    if (success)
    {
        moveit_msgs::MoveItErrorCodes error_codes = group->execute(plan);
        ROS_INFO("%d", error_codes.val);
        return 1;
    }
    else
    {
        return 0;
    }
    return 1;
}

void initPoses(double offset)
{
    pickBase.position.x = 0.148;
    pickBase.position.y = 0.260 + offset;
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

    home.position.x = 0;
    home.position.y = 0.31;
    home.position.z = 0.39;
    home.orientation.w = 0.0;
    home.orientation.x = 0.0;
    home.orientation.y = 1.0;
    home.orientation.z = 0.0;
}

bool montageCallback(cell_core::montage_service::Request &req, cell_core::montage_service::Response &res)
{
    if (req.Offset <= 0.1 && req.Offset >= -0.1)
    {
        if (idle_)
        {
            idle_ = false;

            initPoses(req.Offset);
            ROS_INFO("Startet Service");
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            ROS_INFO("pickFHull");
            MoveToPose(pickFHull, my_plan);
            MoveLinear(0.0, 0.0, -0.05, my_plan);

            ROS_INFO("montage");
            MoveToPose(montage, my_plan);
            MoveLinear(-0.05, 0.0, -0.05, my_plan);

            ROS_INFO("pickTool: ");
            MoveToPose(pickTool, my_plan);

            ROS_INFO("montage");
            MoveToPose(montage, my_plan);

            ROS_INFO("pickSpring");
            MoveToPose(pickSpring, my_plan);
            MoveLinear(0.0, 0.0, -0.05, my_plan);

            ROS_INFO("montage");
            MoveToPose(montage, my_plan);

            ROS_INFO("pickInk");
            MoveToPose(pickInk, my_plan);
            MoveLinear(0.0, 0.0, -0.05, my_plan);

            ROS_INFO("montage");
            MoveToPose(montage, my_plan);

            ROS_INFO("pickArr");
            MoveToPose(pickArr, my_plan);
            MoveLinear(0.0, 0.0, -0.05, my_plan);

            ROS_INFO("montage");
            MoveToPose(montage, my_plan);

            // TODO: circMove

            ROS_INFO("pickRHull");
            MoveToPose(pickRHull, my_plan);
            MoveLinear(0.0, 0.0, -0.05, my_plan);

            ROS_INFO("montageRHull");
            MoveToPose(montageRHull, my_plan);

            ROS_INFO("MOVE Home: ");
            MoveToPose(home, my_plan);
            /*
            if (req.Ausgabestelle == 1)
            {
                MoveToPose(Ausgabe1, my_plan);
                // Greiffer öffnen
            }
            else if (req.Ausgabestelle == 2)
            {
                MoveToPose(Ausgabe2, my_plan);
            }*/
            res.status = 2;
            idle_ = true;
            return 1;
        }
        else
        {
            ROS_INFO("PenMontage is not idle!");
            res.status = 3;
            return 0;
        }
    }
    else
    {
        res.status = 0;
        return 0;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "penAssembly");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2); // Define Multithreadedspinner
    spinner.start();

    group.reset(new moveit::planning_interface::MoveGroupInterface("gripper"));
    group->setPlannerId("RRTkConfigDefault");
    group->setPlanningTime(2);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Advertise Montageservice at ROS-Master
    ros::ServiceServer service = nh.advertiseService("montage_service", montageCallback);
    ROS_INFO("Montage Service rdy!");

    // Publish the Status updater
    ros::Publisher penAssembly_pub = nh.advertise<cell_core::status_msg>("status_chatter", 1000);

    sleep(20);
    collisionObjectAdder coAdder;
    coAdder.addCell(group);

    // Publish the State of the Assembly
    ros::Rate loop_rate(10); //Freq of 10 Hz
    while (ros::ok())
    {
        cell_core::status_msg msg;
        msg.idle = idle_;
        msg.error = error_;
        penAssembly_pub.publish(msg);
        loop_rate.sleep();
    }
    ros::waitForShutdown();
}