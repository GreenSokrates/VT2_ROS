#include <cell_core/penAssembly.h>
#include <tf/transform_datatypes.h>
using namespace std;

bool moveLinear(double x, double y, double z, moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    std::vector<geometry_msgs::Pose> waypoints_tool;
    geometry_msgs::PoseStamped tempStampPose = group->getCurrentPose(group->getEndEffectorLink());
    geometry_msgs::Pose test_pose = tempStampPose.pose;

    test_pose.position.x += x;
    test_pose.position.y += y;
    test_pose.position.z += z;
    waypoints_tool.push_back(test_pose);

    moveit_msgs::RobotTrajectory trajectory_msg;
    double fraction = group->computeCartesianPath(waypoints_tool,
                                                  0.001, //eef_step
                                                  5,     // jump_threshold
                                                  trajectory_msg, true);
    plan.trajectory_ = trajectory_msg;
    ROS_INFO("Visualizing Cartesian Path (%2f%% acheived)", fraction * 100.0);
    if (fraction >= 0.90)
    {
        group->execute(plan);
        return 1;
    }
    else
    {
        return 0;
    }
    return 0;
}

bool moveLinear(geometry_msgs::Pose &pose, moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    std::vector<geometry_msgs::Pose> waypoints_tool;
    geometry_msgs::PoseStamped tempStampPose = group->getCurrentPose(group->getEndEffectorLink());
    //geometry_msgs::Pose start_pose = tempStampPose.pose;

    waypoints_tool.push_back(pose);

    moveit_msgs::RobotTrajectory trajectory_msg;
    double fraction = group->computeCartesianPath(waypoints_tool,
                                                  0.001, //eef_step
                                                  5,     // jump_threshold
                                                  trajectory_msg, true);
    plan.trajectory_ = trajectory_msg;
    ROS_INFO("Visualizing Cartesian Path (%2f%% acheived)", fraction * 100.0);
    if (fraction >= 0.90)
    {
        group->execute(plan);
        return 1;
    }
    else
    {
        return 0;
    }
    return 0;
}
void movePen(moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    std::vector<geometry_msgs::Pose> waypoints_tool;
    geometry_msgs::PoseStamped tempStampPose = group->getCurrentPose(group->getEndEffectorLink());
    geometry_msgs::Pose test_pose = tempStampPose.pose;

    //Move straight up a little
    test_pose.position.z += 0.02;
    waypoints_tool.push_back(test_pose);
    // Move circular
    for (double i = 0.75 * pi_; i >= 0.5 * pi_; i -= 0.01)
    {
        test_pose.position.x = test_pose.position.x - radius_ * (cos(i) - cos(i + 0.1));
        test_pose.position.z = test_pose.position.z + radius_ * (-sin(i) + sin(i - 0.1));
        waypoints_tool.push_back(test_pose);
        ROS_INFO("x: %f", test_pose.position.x);
        ROS_INFO("y: %f", test_pose.position.y);
    }

    moveit_msgs::RobotTrajectory trajectory_msg;
    double fraction = group->computeCartesianPath(waypoints_tool,
                                                  0.01, //eef_step
                                                  2,    // jump_threshold
                                                  trajectory_msg, true);
    plan.trajectory_ = trajectory_msg;
    ROS_INFO("Visualizing MovePen Path (%2f%% acheived)", fraction * 100.0);
    sleep(5.0);
    group->execute(plan);
    sleep(2);
    return;
}

void rotateZ(moveit::planning_interface::MoveGroupInterface::Plan &plan, double angle)
{
    std::vector<double> group_variable_values;
    group->getCurrentState()->copyJointGroupPositions(group->getCurrentState()->getRobotModel()->getJointModelGroup(group->getName()), group_variable_values);
    group_variable_values[5] += angle;
    group->setJointValueTarget(group_variable_values);
    group->plan(plan);
    group->execute(plan);
}

/*
 * Moves arm to given Pose, offset is possible.  !! Offset is in meters !!
 */
bool moveToPose(geometry_msgs::Pose &position, moveit::planning_interface::MoveGroupInterface::Plan &plan, double x = 0, double y = 0, double z = 0)
{
    geometry_msgs::Pose tempPose = position;
    tempPose.position.x += x;
    tempPose.position.y += y;
    tempPose.position.z += z;

    group->setPoseTarget(tempPose);
    if (group->plan(plan))
    {
        moveit_msgs::MoveItErrorCodes error_codes = group->execute(plan);
        ROS_INFO("%d", error_codes.val);
        return 1;
    }
    else
    {
        ROS_WARN("Try with longer Planningtime");
        group->setPlanningTime(10.0);
        if (group->plan(plan))

            return 1;
        else
        {
            group->setPlanningTime(2.0);
            return 0;
        }
    }
    return -1;
}

void initPoses(double offset)
{
    pickBase.position.x = 0.154;
    pickBase.position.y = 0.264 + offset;
    pickBase.position.z = 0.075;
    pickBase.orientation.w = -0.0;
    pickBase.orientation.x = 0.0;
    pickBase.orientation.y = 1.0;
    pickBase.orientation.z = 0.0;

    pickFHull = pickBase;
    pickFHull.position.x = 0.18193;
    pickFHull.position.y = 0.29004 + offset;
    pickFHull.position.z = 0.05808;

    montageFHull.position.x = -0.05599;
    montageFHull.position.y = 0.47872;
    montageFHull.position.z = 0.064198;
    montageFHull.orientation.w = 0.653;
    montageFHull.orientation.x = -0.653;
    montageFHull.orientation.y = 0.271;
    montageFHull.orientation.z = 0.271;

    pickSpring.position.x = 0.17157;
    pickSpring.position.y = 0.42777 + offset;
    pickSpring.position.z = 0.05927;
    pickSpring.orientation.w = 0.0;
    pickSpring.orientation.x = 0.707;
    pickSpring.orientation.y = 0.707;
    pickSpring.orientation.z = 0.0;

    montageSpring.position.x = 0.05582;
    montageSpring.position.y = 0.48100;
    montageSpring.position.z = 0.13717;
    montageSpring.orientation.w = 0.653;
    montageSpring.orientation.x = -0.653;
    montageSpring.orientation.y = 0.271;
    montageSpring.orientation.z = 0.271;

    pickInk.position.x = 0.22012;
    pickInk.position.y = 0.31858 + offset;
    pickInk.position.z = 0.05752;
    pickInk.orientation.w = 0.000;
    pickInk.orientation.x = 0.000;
    pickInk.orientation.y = 1.000;
    pickInk.orientation.z = 0.000;

    montageInk.position.x = -0.02881;
    montageInk.position.y = 0.47867;
    montageInk.position.z = 0.15455;
    montageInk.orientation.w = 0.653;
    montageInk.orientation.x = -0.653;
    montageInk.orientation.y = 0.271;
    montageInk.orientation.z = 0.271;

    pickArr.position.x = 0.22925;
    pickArr.position.y = 0.42786 + offset;
    pickArr.position.z = 0.05866;
    pickArr.orientation.w = 0.0;
    pickArr.orientation.x = 0.707;
    pickArr.orientation.y = 0.707;
    pickArr.orientation.z = 0.0;

    montageArr.position.x = 0.05582;
    montageArr.position.y = 0.48100;
    montageArr.position.z = 0.13717;
    montageArr.orientation.w = 0.653;
    montageArr.orientation.x = -0.653;
    montageArr.orientation.y = 0.271;
    montageArr.orientation.z = 0.271;

    pickRHull.position.x = 0.19800;
    pickRHull.position.y = 0.39300 + offset;
    pickRHull.position.z = 0.05800;
    pickRHull.orientation.w = 0.0;
    pickRHull.orientation.x = 0.707;
    pickRHull.orientation.y = -0.707;
    pickRHull.orientation.z = 0.0;

    montageRHull.position.x = -0.166;
    montageRHull.position.y = 0.594;
    montageRHull.position.z = 0.188;
    montageRHull.orientation.w = 0.271;
    montageRHull.orientation.x = -0.271;
    montageRHull.orientation.y = -0.653;
    montageRHull.orientation.z = -0.653;

    home.position.x = 0;
    home.position.y = 0.31;
    home.position.z = 0.39;
    home.orientation.w = 0.0;
    home.orientation.x = 0.0;
    home.orientation.y = 1.0;
    home.orientation.z = 0.0;

    pickTool.position.x = -0.20101;
    pickTool.position.y = 0.39198;
    pickTool.position.z = 0.064198;
    pickTool.orientation.w = -0.271;
    pickTool.orientation.x = -0.653;
    pickTool.orientation.y = 0.653;
    pickTool.orientation.z = 0.271;

    useTool.position.x = -0.0235;
    useTool.position.y = 0.5;
    useTool.position.z = 0.13228;
    useTool.orientation.w = -0.271;
    useTool.orientation.x = -0.653;
    useTool.orientation.y = 0.653;
    useTool.orientation.z = 0.271;

    underPen.position.x = -0.02;
    underPen.position.y = 0.5;
    underPen.position.z = 0.065;
    underPen.orientation.w = 0.566;
    underPen.orientation.x = -0.591;
    underPen.orientation.y = 0.389;
    underPen.orientation.z = 0.421;
}

bool montageCallback(cell_core::montage_service::Request &req, cell_core::montage_service::Response &res)
{
    if (req.Offset <= 0.1 && req.Offset >= -0.1)
    {
        if (true)
        {
            idle_ = false;
            initPoses(req.Offset);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            moveToPose(home, my_plan);

            // -- FrontHull --
            ROS_INFO("Front Hull");
            if (!moveToPose(pickFHull, my_plan, 0, 0, zPickOffset))
            {
                ROS_ERROR("Can't reach Pose pickoffsetHull");
                return -1;
            }
            moveLinear(pickFHull, my_plan);
            coAdder->addBody(group, 2);
            // Gripper close
            moveLinear(0.0, 0.0, 0.07, my_plan);

            if (!moveToPose(montageFHull, my_plan, 0.055, 0, 0.055))
            {
                ROS_ERROR("Can't reach Pose montageFHull");
                return -1;
            }
            moveLinear(montageFHull, my_plan);
            coAdder->detatchBody(group, 2);
            // close 3bGripper a little
            //Gripper open
            moveLinear(0.06, 0.0, 0.06, my_plan);

            // -- Pick Tool --
            ROS_INFO("Pick Tool, turn and place Tool");
            if (!moveToPose(pickTool, my_plan, -0.03, 0, 0.03))
            {
                ROS_ERROR("Can't reach Pose pickTool");
                return -1;
            }
            moveLinear(pickTool, my_plan);
            //Gripper close
            moveLinear(0.045, 0.0, 0.045, my_plan);

            if (!moveToPose(useTool, my_plan, 0.01, 0.0, 0.01))
            {
                ROS_ERROR("Can't reach Pose useTool");
                return -1;
            }
            moveLinear(-0.01, 0.0, -0.01, my_plan);

            // -- Rotate tcp--
            ROS_INFO("Begin rotate");
            rotateZ(my_plan, 6.28);
            moveLinear(0.05, 0.0, 0.05, my_plan);

            // -- Return Tool --
            if (!moveToPose(pickTool, my_plan, 0.045, 0, 0.045))
            {
                ROS_ERROR("Can't reach Pose placeTool");
                return -1;
            }
            moveLinear(0.044, 0.0, -0.044, my_plan);
            //Gripper open
            moveLinear(-0.015, 0.0, 0.015, my_plan);
            moveLinear(0.02, 0, 0.02, my_plan);

            group->setEndEffectorLink("small_grasping_frame");

            // -- SPRING --
            ROS_INFO("Spring");
            if (!moveToPose(pickSpring, my_plan, 0, 0, zPickOffset))
            {
                ROS_ERROR("Can't reach Pose pickSpring");
                return -1;
            }
            moveLinear(pickSpring, my_plan);
            // Gripper close
            moveLinear(0.0, 0.0, -zPickOffset, my_plan);
            if (!moveToPose(montageSpring, my_plan, 0.020, 0, 0.020))
            {
                ROS_ERROR("Can't reach Pose montageSpring");
                return -1;
            }
            moveLinear(montageSpring, my_plan);
            //Gripper open
            moveLinear(0.05, 0.0, 0.05, my_plan);

            // -- INK --
            ROS_INFO("Ink");
            if (!moveToPose(pickInk, my_plan, 0, 0, zPickOffset))
            {
                ROS_ERROR("Can't reach Pose pickInk");
                return -1;
            }
            moveLinear(pickInk, my_plan);
            // Gripper close
            moveLinear(0.0, 0.0, -zPickOffset, my_plan);
            if (!moveToPose(montageInk, my_plan, 0.050, 0, 0.050))
            {
                ROS_ERROR("Can't reach Pose montageInk");
                return -1;
            }
            moveLinear(montageInk, my_plan);
            //Gripper open
            moveLinear(0.05, 0.0, 0.05, my_plan);

            // -- Turn pen 45° --
            ROS_INFO("Pen Aufrichten");
            if (!moveToPose(underPen, my_plan, 0, 0.05, 0))
            {
                ROS_ERROR("Can't reach Pose underPEN");
                return -1;
            }
            moveLinear(underPen, my_plan);
            movePen(my_plan);
            moveLinear(0.05, 0.0, 0.0, my_plan);

            // -- Arretierung --
            ROS_INFO("Pick Arretierung");
            if (!moveToPose(pickArr, my_plan, 0, 0, zPickOffset))
            {
                ROS_ERROR("Can't reach Pose pickInk");
                return -1;
            }
            moveLinear(pickArr, my_plan);
            //Gripper close
            moveLinear(0.0, 0.0, -zPickOffset, my_plan);
            if (!moveToPose(montageArr, my_plan, 0.0, 0.0, 0.02))
            {
                ROS_ERROR("Can't reach");
            }
            moveLinear(montageArr, my_plan);
            //Gripper open
            moveLinear(0.0, 0.0, 0.02, my_plan);

            // -- Deckel --
            ROS_INFO("Pick Deckel");
            if (!moveToPose(pickRHull, my_plan, 0, 0, zPickOffset))
            {
                ROS_ERROR("Can't reach Pose pickInk");
                return -1;
            }
            moveLinear(pickRHull, my_plan);
            // close Gripper
            moveLinear(0.0, 0.0, -zPickOffset, my_plan);
            if (!moveToPose(montageRHull, my_plan, 0.0, 0.0, 0.010))
            {
                ROS_ERROR("Can't reach Pose montageRHull");
                return -1;
            }
            moveLinear(montageRHull, my_plan);

            // -- Pen zurück drehen --

            // -- Pen Ausgeben --
            ROS_INFO("Pen ausgabe");
            if (!moveToPose(montageFHull, my_plan, 0, -0.025, 0.0))
            {
                ROS_ERROR("Can't reach Pose pickInk");
                return -1;
            }
            moveLinear(montageFHull, my_plan);
            moveLinear(0.055, 0.0, 0.055, my_plan);

            moveToPose(penAusgabe, my_plan);

            ROS_INFO("MOVE Home: ");
            moveToPose(home, my_plan);

            res.status = 1;
            idle_ = true;
            return 1;
        }
        else
        {
            ROS_INFO("PenMontage is not idle!");
            res.status = 3;
            return 1;
        }
    }
    else
    {
        res.status = 0;
        return 0;
    }
}

bool moveRobotCallback(cell_core::moveRobotToPose::Request &req, cell_core::moveRobotToPose::Response &res)
{
    geometry_msgs::Pose myPose;
    myPose.position.x = req.x;
    myPose.position.y = req.y;
    myPose.position.z = req.z;
    myPose.orientation.w = req.oW;
    myPose.orientation.x = req.oX;
    myPose.orientation.y = req.oY;
    myPose.orientation.z = req.oZ;
    group->setPoseTarget(myPose);
    group->asyncMove();
    res.status = 1;
    return 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "penAssembly");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0); // Define Multithreadedspinner
    spinner.start();

    // Define MoveGroup and PlanningSceneInterface
    group.reset(new moveit::planning_interface::MoveGroupInterface("gripper"));
    //group->setPlannerId("RRTConnect");
    group->setEndEffectorLink("grasping_frame");
    group->setPlanningTime(1.5);
    group->setGoalJointTolerance(0.0001);
    group->setNumPlanningAttempts(3);
    group->setGoalOrientationTolerance(0.0001);
    group->setMaxVelocityScalingFactor(0.1);

    // Advertise Services at ROS-Master
    ros::ServiceServer service = nh.advertiseService("/penAssembly/montage_service", montageCallback);
    ros::ServiceServer moveservice = nh.advertiseService("/penAssembly/moveRobotToPose", moveRobotCallback);
    ROS_INFO("Service rdy!");

    // Publish the Status updater
    ros::Publisher penAssembly_pub = nh.advertise<cell_core::status_msg>("/penAssembly/status", 1000);

    collisionObject coAdder;
    sleep(15); // to make sure move_group is up
    coAdder.addCell(group);

    sleep(10);

    ROS_INFO("Node Ready!");
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
    return 1;
}