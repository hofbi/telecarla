#include "telecarla_rpc_client.h"

#include <ros/ros.h>

using namespace lmt::client;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "telecarla_rpc_client");

    ros::NodeHandle nh = ros::NodeHandle();
    ros::NodeHandle pnh = ros::NodeHandle("~");

    TeleCarlaRpcClient client(nh, pnh);

    const auto loop_frequency_hz{50.0};
    ros::Rate loop(loop_frequency_hz);

    while (ros::ok())
    {
        client.update();
        ros::spinOnce();
        loop.sleep();
    }

    return EXIT_SUCCESS;
}
