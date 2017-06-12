#include <ros/ros.h>
#include <http_server/HTTPServer.h>

using namespace ros;

int main(int argc, char **argv)
{
    init(argc, argv, "HTTP_server");
    NodeHandle nh;
    HTTPServer *httpServer = new HTTPServer(8080);
    httpServer->start();
    AsyncSpinner spinner(0);

    spin(); //Keeps node alive
}