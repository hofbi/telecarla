#include <ros/ros.h>

#include "telecarla_gui.h"

using namespace lmt;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "telecarla_gui");

    ros::NodeHandle nh = ros::NodeHandle();
    ros::NodeHandle pnh = ros::NodeHandle("~");

    TeleCarlaGui gui(nh, pnh);

    const auto loopFrequencyInHz{50.0};
    ros::Rate loop(loopFrequencyInHz);

    while (ros::ok())
    {
        ros::spinOnce();
        gui.update();
        loop.sleep();
    }

    return 0;
}
