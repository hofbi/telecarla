#include "bool_msg_callback.h"

#include <ros/console.h>
#include <rpc/client.h>
#include <rpc/rpc_error.h>

using namespace lmt::client;

BoolMsgCallback::BoolMsgCallback(rpc::client& client, std::string functionName) noexcept
    : client_(client), functionName_(std::move(functionName))
{
}

void BoolMsgCallback::operator()(const std_msgs::BoolConstPtr& boolConstPtr)
{
    try
    {
        client_.send(functionName_, static_cast<bool>(boolConstPtr->data));
    }
    catch (rpc::rpc_error& error)
    {
        ROS_WARN_STREAM("Failed to call function " << error.get_function_name() << " with "
                                                   << error.get_error().as<std::string>());
    }
}
