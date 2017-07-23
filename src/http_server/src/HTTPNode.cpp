#include <http_server/HTTPNode.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "HTTP_server");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2); // Define Multithreadedspinner
    spinner.start();

    HTTPServer *httpServer = new HTTPServer(8080);
    HTTPServer *httpServer2 = new HTTPServer(8081);
    httpServer->start();
    httpServer2->start();
    httpServer->add("montage", new HTTPScript_montage());
    httpServer2->add("status", new HTTPScript_status());

    ROS_INFO("Http-Server is running!");

    while(ros::ok());

    return 0;
}