#include "bool_callback.h"

#include <std_msgs/Bool.h>

using namespace lmt::server;

void BoolCallback::operator()(bool value) const
{
    std_msgs::Bool boolMsg;
    boolMsg.data = value;
    msgPublisher_.publish(boolMsg);
}

BoolCallback::BoolCallback(const ros::Publisher& msgPublisher) : msgPublisher_(msgPublisher) {}
