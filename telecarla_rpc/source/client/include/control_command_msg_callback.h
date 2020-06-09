#ifndef TELECARLA_RPC_CONTROL_COMMAND_MSG_CALLBACK_H
#define TELECARLA_RPC_CONTROL_COMMAND_MSG_CALLBACK_H

#include <carla_msgs/CarlaEgoVehicleControl.h>

namespace rpc
{
class client;
}
namespace lmt
{
namespace client
{
class ControlCommandMsgCallback
{
  public:
    ControlCommandMsgCallback(rpc::client& client, std::string functionName) noexcept;

    void operator()(const carla_msgs::CarlaEgoVehicleControlConstPtr& controlMsg);

  private:
    rpc::client& client_;
    std::string functionName_;
};
}  // namespace client
}  // namespace lmt

#endif  // TELECARLA_RPC_CONTROL_COMMAND_MSG_CALLBACK_H
