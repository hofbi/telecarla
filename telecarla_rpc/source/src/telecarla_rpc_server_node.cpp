#include <ros/ros.h>

#include "telecarla_rpc_server.h"

using namespace lmt::server;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "telecarla_rpc_server");

    ros::NodeHandle nh = ros::NodeHandle();
    ros::NodeHandle pnh = ros::NodeHandle("~");

    TeleCarlaRpcServer server(nh, pnh);

    const auto loopFrequencyInHz{50.0};
    ros::Rate loop(loopFrequencyInHz);
    server.run();

    while (ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }

    server.stop();

    return EXIT_SUCCESS;
}
