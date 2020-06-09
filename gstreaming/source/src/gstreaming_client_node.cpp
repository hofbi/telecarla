#include "gstreaming_client.h"

#include <ros/ros.h>

using namespace lmt;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "rtsp_client");

    ros::NodeHandle nh = ros::NodeHandle();
    ros::NodeHandle pnh = ros::NodeHandle("~");

    GStreamingClient clientRos(nh, pnh, argc, argv);

    ros::Rate loop(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }

    return EXIT_SUCCESS;
}
