#ifndef TELECARLA_RPC_CONTROL_COMMANDS_H
#define TELECARLA_RPC_CONTROL_COMMANDS_H

#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <rpc/msgpack.hpp>

namespace lmt
{
namespace common
{
class ControlCommands
{
  public:
    ControlCommands() = default;
    explicit ControlCommands(const carla_msgs::CarlaEgoVehicleControlConstPtr& controlMsg) noexcept;
    explicit ControlCommands(const carla_msgs::CarlaEgoVehicleControl& controlMsg) noexcept;

    carla_msgs::CarlaEgoVehicleControl getMessage() const noexcept;

    MSGPACK_DEFINE_MAP(throttle, steer, brake, hand_brake, reverse);

  private:
    float throttle;
    float steer;
    float brake;
    bool hand_brake;
    bool reverse;
};
}  // namespace common
}  // namespace lmt

#endif  // TELECARLA_RPC_CONTROL_COMMANDS_H
