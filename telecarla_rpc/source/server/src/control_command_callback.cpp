#include "control_command_callback.h"

#include "control_commands.h"

using namespace lmt::server;

ControlCommandCallback::ControlCommandCallback(const ros::Publisher& msgPublisher) : msgPublisher_(msgPublisher) {}

void ControlCommandCallback::operator()(const common::ControlCommands& controlCommands) const
{
    msgPublisher_.publish(controlCommands.getMessage());
}
