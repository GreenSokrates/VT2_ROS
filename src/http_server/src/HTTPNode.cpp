#include <ros/ros.h>
#include <http_server/HTTPServer.h>
#include "std_msgs/String.h"
#include <http_server/http_msg.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "HTTP_server");
    ros::NodeHandle nh;
    HTTPServer *httpServer = new HTTPServer(8080);
    httpServer->start();

    ros::Publisher http_pub = nh.advertise<http_server::http_msg>("http_msg", 1000);
    ros::Rate loop_rate(5);

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