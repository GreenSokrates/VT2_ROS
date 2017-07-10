#include <ros/ros.h>
#include <http_server/HTTPServer.h>
#include "std_msgs/String.h"
#include <http_server/http_msg.h>

void chatterCallback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "HTTP_server");
    ros::NodeHandle nh;
    HTTPServer *httpServer = new HTTPServer(8080);
    httpServer->start();

    ros::Publisher http_pub = nh.advertise<http_server::http_msg>("http_msg/topics/push", 1000);
    ros::Rate loop_rate(5);

    ros::Subscriber sub = nh.subscribe("/http_msg/topics/status", 1000, chatterCallback);

    int count = 0;
    while (ros::ok())
    {
        http_server::http_msg msg;
        msg.header.stamp = ros::Time::now();
        msg.task = 1;
        msg.x = 0.0;
        msg.y = 0.0;

        http_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}