#ifndef TELECARLA_RPC_CONTROL_COMMAND_CALLBACK_H
#define TELECARLA_RPC_CONTROL_COMMAND_CALLBACK_H

#include <ros/publisher.h>

namespace lmt
{
namespace common
{
class ControlCommands;
}
namespace server
{
class ControlCommandCallback
{
  public:
    explicit ControlCommandCallback(const ros::Publisher& msgPublisher);

    void operator()(const common::ControlCommands& controlCommands) const;

  private:
    ros::Publisher msgPublisher_;
};
}  // namespace server
}  // namespace lmt

#endif  // TELECARLA_RPC_CONTROL_COMMAND_CALLBACK_H
