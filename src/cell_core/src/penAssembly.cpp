#include <cell_core/penAssembly.h>
#include <tf/transform_datatypes.h>
using namespace std;

bool MoveLinear(double x, double y, double z, moveit::planning_interface::MoveGroupInterface::Plan &plan)
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
                                                  5,   // jump_threshold
                                                  trajectory_msg, true);
    plan.trajectory_ = trajectory_msg;
    ROS_INFO("Visualizing Cartesian Path (%2f%% acheived)", fraction * 100.0);
    if (fraction >= 0.85)
    {
        group->execute(plan);
        return 1;
    }
    else
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
    geometry_msgs::Pose start_pose = test_pose;

    // Move circular
    for (double i = 0.75 * pi_; i <= 0.5 * pi_; i += 0.01)
    {
        test_pose.position.x = start_pose.position.x + radius_ * cos(i);
        test_pose.position.z = start_pose.position.z + radius_ * sin(i);
        waypoints_tool.push_back(test_pose);
    }

    moveit_msgs::RobotTrajectory trajectory_msg;
    double fraction = group->computeCartesianPath(waypoints_tool,
                                                  0.001, //eef_step
                                                  0,   // jump_threshold
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
    group_variable_values[5] = angle;
    group->setJointValueTarget(group_variable_values);
    group->plan(plan);
    group->execute(plan);
}

bool MoveToPose(geometry_msgs::Pose &position, moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    group->setPoseTarget(position);
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
    pickBase.position.x = 0.148;
    pickBase.position.y = 0.260 + offset;
    pickBase.position.z = 0.100;
    pickBase.orientation.w = -0.017;
    pickBase.orientation.x = 0.0;
    pickBase.orientation.y = 1.0;
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
    pickSpring.orientation.w = 0.0;
    pickSpring.orientation.x = 0.707;
    pickSpring.orientation.y = 0.707;
    pickSpring.orientation.z = 0.0;

    pickArr = pickBase;
    pickArr.position.x += 0.0705;
    pickArr.position.y += 0.172;
    pickArr.position.z += 0.000;
    pickArr.orientation.w = 0.0;
    pickArr.orientation.x = 0.707;
    pickArr.orientation.y = 0.707;
    pickArr.orientation.z = 0.0;

    home.position.x = 0;
    home.position.y = 0.31;
    home.position.z = 0.39;
    home.orientation.w = 0.0;
    home.orientation.x = 0.0;
    home.orientation.y = 1.0;
    home.orientation.z = 0.0;

    pickTool.position.x = -0.23601;
    pickTool.position.y = 0.45;
    pickTool.position.z = 0.29;
    pickTool.orientation.w = 0.0;
    pickTool.orientation.x = -0.0;
    pickTool.orientation.y = -0.924;
    pickTool.orientation.z = -0.383;

    useTool.position.x = -0.0235;
    useTool.position.y = 0.57372;
    useTool.position.z = 0.13228;
    useTool.orientation.w = -0.257;
    useTool.orientation.x = -0.636;
    useTool.orientation.y = 0.671;
    useTool.orientation.z = 0.278;

    underPen.position.x = -0.033;
    underPen.position.y = 0.601;
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
        if (idle_)
        {
            idle_ = false;
            initPoses(req.Offset);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            //MoveToPose(home, my_plan);

            ROS_INFO("Front Hull");
            MoveToPose(pickFHull, my_plan);
            MoveLinear(0.0, 0.0, -0.05, my_plan);
            // Gripper close
            MoveLinear(0.0, 0.0, 0.05, my_plan);
            MoveToPose(montage, my_plan);
            MoveLinear(-0.05, 0.0, -0.05, my_plan);
            // 3bGripper close
            //Gripper open
            MoveLinear(0.05, 0.0, 0.05, my_plan);

            ROS_INFO("Pick Tool, turn and place Tool");
            MoveToPose(pickTool, my_plan);
            MoveLinear(0.0, 0.05, -0.05, my_plan);
            MoveLinear(0.0, -0.01, -0.01, my_plan);
            //Gripper close
            MoveLinear(0.0, -0.05, 0.05, my_plan);
            MoveToPose(useTool, my_plan);
            rotateZ(my_plan, -1.57);
            MoveLinear(-0.05, 0.0, -0.05, my_plan);
            //Rotate tcp
            ROS_INFO("Begin rotate");
            rotateZ(my_plan, 1.57);

            MoveLinear(0.05, 0.0, 0.05, my_plan);
            MoveToPose(pickTool, my_plan);
            MoveLinear(0.0, 0.05, -0.05, my_plan);
            //Gripper open
            MoveLinear(0.0, 0.01, 0.01, my_plan);
            MoveLinear(0.0, -0.05, 0.05, my_plan);

            ROS_INFO("Spring");
            MoveToPose(pickSpring, my_plan);
            MoveLinear(0.0, 0.0, -0.05, my_plan);
            // Gripper close
            MoveLinear(0.0, 0.0, 0.05, my_plan);
            MoveToPose(montage, my_plan);
            MoveLinear(-0.05, 0.0, -0.05, my_plan);
            //Gripper open
            MoveLinear(0.05, 0.0, 0.05, my_plan);

            ROS_INFO("Ink");
            MoveToPose(pickInk, my_plan);
            MoveLinear(0.0, 0.0, -0.05, my_plan);
            // Gripper close
            MoveLinear(0.0, 0.0, 0.05, my_plan);
            MoveToPose(montage, my_plan);
            MoveLinear(-0.05, 0.0, -0.05, my_plan);
            //Gripper open
            MoveLinear(0.05, 0.0, 0.05, my_plan);

            ROS_INFO("Pen Aufrichten");
            MoveToPose(underPen, my_plan);
            MoveLinear(0.0, 0.05, 0.0, my_plan);
            movePen(my_plan);

            // -- Arretierung --

            // -- Deckel --

            // -- Pen zurück drehen

            // -- Pen Ausgeben

            ROS_INFO("MOVE Home: ");
            MoveToPose(home, my_plan);

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

    // Define MoveGroup and PlanningSceneInterface
    group.reset(new moveit::planning_interface::MoveGroupInterface("gripper"));
    group->setPlannerId("RRTkConfigDefault");
    group->setEndEffectorLink("grasping_frame");
    group->setPlanningTime(2);
    group->setGoalJointTolerance(0.01);
    group->setNumPlanningAttempts(3);
    group->setGoalOrientationTolerance(0.01);
    group->setMaxVelocityScalingFactor(0.1);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Advertise Montageservice at ROS-Master
    ros::ServiceServer service = nh.advertiseService("montage_service", montageCallback);
    ROS_INFO("Montage Service rdy!");

    // Publish the Status updater
    ros::Publisher penAssembly_pub = nh.advertise<cell_core::status_msg>("status_chatter", 1000);

    sleep(15);
    collisionObjectAdder coAdder;
    planning_scene_interface.addCollisionObjects(coAdder.addCell(group));

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