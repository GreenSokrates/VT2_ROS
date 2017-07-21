#include <moveit_msgs/CollisionObject.h>
#include <cell_core/collisionObject.h>

using namespace std;

collisionObjectAdder::collisionObjectAdder()
{
    add_collision_object_pub = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 1000);
}

void collisionObjectAdder::addCell(boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> &group)
{
    // Generating Collision object from Mesh
    Eigen::Vector3d scaling_vector(0.001, 0.001, 0.001); // Scaling Vector
    moveit_msgs::CollisionObject co;
    co.header.frame_id = group->getPlanningFrame();
    co.id = "WorkCell";
    shapes::Mesh *m = shapes::createMeshFromResource("package://cell_support/meshes/Mittelteil_final_collision.stl", scaling_vector);
    ROS_INFO("Mesh Loaded");

    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

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

    co.meshes.push_back(mesh);
    co.mesh_poses.push_back(co.mesh_poses[0]);
    co.operation = co.ADD;

    add_collision_object_pub.publish(co);
    ROS_INFO("Collision object published");
}