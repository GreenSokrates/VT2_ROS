#include <moveit_msgs/CollisionObject.h>
#include <cell_core/collisionObject.h>

using namespace std;

collisionObject::collisionObject()
{
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    //add_collision_object_pub = nh.advertise<moveit_msgs::CollisionObject>("/collision_object", 1000);
}

void collisionObject::addCell(boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> &group)
{
    // Generating Collision object from Mesh
    Eigen::Vector3d scaling_vector(0.001, 0.001, 0.001); // Scaling Vector
    moveit_msgs::CollisionObject co;
    co.header.frame_id = group->getPlanningFrame();
    co.id = "WorkCell";
    shapes::Mesh *m = shapes::createMeshFromResource("package://cell_support/meshes/1000_Anlage_collision.stl", scaling_vector);
    ROS_INFO("Mesh Loaded");

    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    // Define Size and orientation
    co.meshes.resize(1);
    co.mesh_poses.resize(1);
    co.meshes[0] = mesh;
    co.mesh_poses[0].position.x = -1.075;
    co.mesh_poses[0].position.y = 0.023;
    co.mesh_poses[0].position.z = -0.021;
    co.mesh_poses[0].orientation.w = 0.707;
    co.mesh_poses[0].orientation.x = 0.0;
    co.mesh_poses[0].orientation.y = 0.0;
    co.mesh_poses[0].orientation.z = 0.707;

    // Push onto vector and return it
    co.meshes.push_back(mesh);
    co.mesh_poses.push_back(co.mesh_poses[0]);
    co.operation = co.ADD;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(co);
    planning_scene_interface.addCollisionObjects(collision_objects);
    return;
}

void collisionObject::addBody(boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> &group, int penPart)
{
    //MESH
    moveit_msgs::CollisionObject co;
    Eigen::Vector3d scaling_vector(0.001, 0.001, 0.001);
    co.header.frame_id = group->getPlanningFrame();
    shapes::Mesh *m;
    switch (penPart)
    {
    case 1:
        co.id = "fHull";
        m = shapes::createMeshFromResource("package://cell_support/meshes/0001_DreamPen_Deckel.stl", scaling_vector);
        break;
    case 2:
        co.id = "rHull";
        m = shapes::createMeshFromResource("package://cell_support/meshes/0003_DreamPen_Gehaeuse.stl", scaling_vector);
        break;
    case 3:
        co.id = "Ink";
        m = shapes::createMeshFromResource("package://cell_support/meshes/0005_DreamPen_Miene.stl", scaling_vector);
        break;
    case 4:
        co.id = "tool";
        m = shapes::createMeshFromResource("package://cell_support/meshes/4017_Drehwerkzeug.stl", scaling_vector);
        break;
    }
    geometry_msgs::PoseStamped curPose = group->getCurrentPose();
    shape_msgs::Mesh co_mesh;
    shapes::ShapeMsg co_mesh_msg;
    shapes::constructMsgFromShape(m, co_mesh_msg);
    co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);
    co.meshes.resize(1);
    co.meshes[0] = co_mesh;
    co.mesh_poses.resize(1);
    co.mesh_poses[0].position.x = curPose.pose.position.x;
    co.mesh_poses[0].position.y = curPose.pose.position.y;
    co.mesh_poses[0].position.z = curPose.pose.position.z;
    co.mesh_poses[0].orientation.w = curPose.pose.orientation.w;
    co.mesh_poses[0].orientation.x = curPose.pose.orientation.x;
    co.mesh_poses[0].orientation.y = curPose.pose.orientation.y;
    co.mesh_poses[0].orientation.z = curPose.pose.orientation.z;
    //pub_co.publish(co);

    co.meshes.push_back(co_mesh);
    co.mesh_poses.push_back(co.mesh_poses[0]);
    co.operation = co.ADD;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(co);

    // Now, let's add the collision object into the world
    planning_scene_interface.addCollisionObjects(collision_objects);
    sleep(5);

    ROS_INFO("Attach the tool to the robot");
    group->attachObject(co.id, group->getEndEffectorLink());
    return;
}

void collisionObject::detatchBody(boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> &group, int penPart)
{
    std::string id;
    switch (penPart)
    {
    case 1:
        id = "fHull";
        break;
    case 2:
        id = "rHull";
        break;
    case 3:
        id = "Ink";
        break;
    case 4:
        id = "tool";
        break;
    }
    group->detachObject(id);
}

void collisionObject::removeBody(boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> &group, int penPart)
{
    moveit_msgs::CollisionObject co;
    std::string id;
    switch (penPart)
    {
    case 1:
        co.id = "fHull";
        break;
    case 2:
        co.id = "rHull";
        break;
    case 3:
        co.id = "Ink";
        break;
    case 4:
        co.id = "tool";
        break;
    }
    std::vector<std::string> collision_objects;
    collision_objects.push_back(co.id);
    group->detachObject(co.id);
    planning_scene_interface.removeCollisionObjects(collision_objects);
}
