#include "control_commands.h"

using namespace lmt::common;

ControlCommands::ControlCommands(const carla_msgs::CarlaEgoVehicleControlConstPtr& controlMsg) noexcept
    : ControlCommands(*controlMsg)
{
}

carla_msgs::CarlaEgoVehicleControl ControlCommands::getMessage() const noexcept
{
    carla_msgs::CarlaEgoVehicleControl carlaEgoVehicleControl;

    carlaEgoVehicleControl.throttle = throttle;
    carlaEgoVehicleControl.brake = brake;
    carlaEgoVehicleControl.steer = steer;
    carlaEgoVehicleControl.reverse = reverse;
    carlaEgoVehicleControl.hand_brake = hand_brake;

    return carlaEgoVehicleControl;
}

ControlCommands::ControlCommands(const carla_msgs::CarlaEgoVehicleControl& controlMsg) noexcept
    : throttle(controlMsg.throttle),
      brake(controlMsg.brake),
      steer(controlMsg.steer),
      reverse(controlMsg.reverse),
      hand_brake(controlMsg.hand_brake)
{
}
