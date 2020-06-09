#ifndef TELECARLA_RPC_VEHICLE_STATUS_H
#define TELECARLA_RPC_VEHICLE_STATUS_H

#include "control_commands.h"

#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <rpc/msgpack.hpp>

namespace lmt
{
namespace common
{
class VehicleStatus
{
  public:
    VehicleStatus() = default;
    explicit VehicleStatus(const carla_msgs::CarlaEgoVehicleStatusConstPtr& statusMsg) noexcept;

    carla_msgs::CarlaEgoVehicleStatus getMessage() const noexcept;

    MSGPACK_DEFINE_MAP(velocity, control);

  private:
    float velocity;
    ControlCommands control;
};
}  // namespace common
}  // namespace lmt

#endif  // TELECARLA_RPC_VEHICLE_STATUS_H
