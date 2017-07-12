#include <cell_core/penAssembly.h>

using namespace ros;
using namespace moveit;

void MoveToPose(geometry_msgs::Pose &position)
{
    group.setPoseTarget(position);
    bool success = group.plan(my_plan);
    if (success)
        group.move();
    return;
}

void MoveLinear(double x, double y, double z)
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
    my_plan.trajectory_ = trajectory_msg;
    ROS_INFO("Visualizing Cartesian Path (%2f%% acheived)", fraction * 100.0);
    sleep(5.0);
    group.execute(my_plan);
    group.setPlanningTime(10.0);
}

void AssemblePen()
{
    MoveToPose(pickFHull);
    MoveLinear(0.0, 0.0, -0.07);
    MoveToPose(montage);
    MoveLinear(-0.05, 0.0, -0.05);
    MoveToPose(pickTool);
    ROS_INFO("pickTool");
    MoveToPose(montage);
    ROS_INFO("montage");
    MoveToPose(pickSpring);
    MoveLinear(0.0, 0.0, -0.07);
    ROS_INFO("pickSpring");
    MoveToPose(montage);
    ROS_INFO("montage");
    MoveToPose(pickInk);
    MoveLinear(0.0, 0.0, -0.07);
    ROS_INFO("pickInk");
    MoveToPose(montage);
    ROS_INFO("montage");
    MoveToPose(pickArr);
    MoveLinear(0.0, 0.0, -0.07);
    ROS_INFO("pickArr");
    MoveToPose(montage);
    ROS_INFO("montage");
    // TODO: circMove
    MoveToPose(pickRHull);
    MoveLinear(0.0, 0.0, -0.07);
    ROS_INFO("pickRHull");
    MoveToPose(montageRHull);
    ROS_INFO("montageRHull");
}

void chatterCallback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());

    /*
    if message --> assemble pen function
    set status to busy, then assemble one pen
    AssemblePen(my_plan, group);
    */
}

bool add(cell_core::montage::Request &req, cell_core::montage::Response &res)
{
    res.status = 1;
    if (req.Ausgabestelle == 1)
    {
        AssemblePen();
    }
    else if (req.Ausgabestelle == 2)
    {
        AssemblePen();
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "penAssembly");
    ros::NodeHandle n;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("gripper_eef");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

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
    montageRHull.position.y = 0.594;
    montageRHull.position.z = 0.188;
    montageRHull.orientation.w = 0.665;
    montageRHull.orientation.x = -0.665;
    montageRHull.orientation.y = -0.238;
    montageRHull.orientation.z = 0.238;

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
    pickSpring.orientation.w = 0.707;
    pickSpring.orientation.x = 0.0;
    pickSpring.orientation.y = 0.0;
    pickSpring.orientation.z = 0.707;

    geometry_msgs::Pose pickArr = pickBase;
    pickArr.position.x += 0.0705;
    pickArr.position.y += 0.172;
    pickArr.position.z += 0.000;
    pickArr.orientation.w = 0.707;
    pickArr.orientation.x = 0.0;
    pickArr.orientation.y = 0.0;
    pickArr.orientation.z = 0.707;

    ros::ServiceServer service = n.advertiseService("montage", &add);
    /*
    ros::Subscriber sub = n.subscribe("/http_msg/topics/push", 1000, chatterCallback);
    ros::Publisher pub = n.advertise<std_msgs::String>("http_msg/topics/status", 1000);
*/
    ros::AsyncSpinner spinner(1);
    spinner.start();

    spinner.stop();
    return (0);
}