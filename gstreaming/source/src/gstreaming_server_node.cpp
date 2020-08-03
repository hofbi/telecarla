#include "gstreaming_server.h"

#include <memory>
#include <ros/ros.h>

using namespace lmt;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "rtsp_server");

    ros::NodeHandle nh = ros::NodeHandle();
    ros::NodeHandle pnh = ros::NodeHandle("~");

    GStreamingServer serverRos(nh, pnh, argc, argv);

    const auto loop_frequency_hz{10.0};
    ros::Rate loop(loop_frequency_hz);

    while (ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }

    return EXIT_SUCCESS;
}
