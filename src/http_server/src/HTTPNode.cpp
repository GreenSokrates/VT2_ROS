#include <http_server/HTTPNode.h>


void statusCallback(const cell_core::status_msg::ConstPtr &msg)
{
    ROS_INFO("HALLO");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "HTTP_server");
    ros::NodeHandle nh;

    HTTPServer *httpServer = new HTTPServer(8080);
    httpServer->start();
    httpServer->add("montage", new HTTPScript_montage());
    httpServer->add("status", new HTTPScript_status());

    ros::Subscriber sub = nh.subscribe("status_chatter", 1000, statusCallback);

    ros::ServiceClient client = nh.serviceClient<cell_core::montage_service>("montage_service");
    clientPtr = &client;
    ROS_INFO("Http-Server is running!");

    ros::spin();

    return 0;
}