#include "control_command_msg_callback.h"

#include <ros/console.h>
#include <rpc/client.h>
#include <rpc/rpc_error.h>

#include "control_commands.h"

using namespace lmt::client;

ControlCommandMsgCallback::ControlCommandMsgCallback(rpc::client& client, std::string functionName) noexcept
    : client_(client), functionName_(std::move(functionName))
{
}

void ControlCommandMsgCallback::operator()(const carla_msgs::CarlaEgoVehicleControlConstPtr& controlMsg)
{
    common::ControlCommands controlCommands(controlMsg);

    try
    {
        client_.send(functionName_, controlCommands);
    }
    catch (rpc::rpc_error& error)
    {
        ROS_WARN_STREAM("Failed to call function " << error.get_function_name() << " with "
                                                   << error.get_error().as<std::string>());
    }
}
