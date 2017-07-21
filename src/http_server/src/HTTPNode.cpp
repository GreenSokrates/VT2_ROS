#include <http_server/HTTPNode.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "HTTP_server");
    ros::NodeHandle nh;

    HTTPServer *httpServer = new HTTPServer(8080);
    HTTPServer *httpServer2 = new HTTPServer(8081);
    httpServer->start();
    httpServer2->start();
    httpServer->add("montage", new HTTPScript_montage());
    httpServer2->add("status", new HTTPScript_status());

    ROS_INFO("Http-Server is running!");

    ros::spin();

    return 0;
}