#include "vehicle_status.h"

using namespace lmt::common;

VehicleStatus::VehicleStatus(const carla_msgs::CarlaEgoVehicleStatusConstPtr& statusMsg) noexcept
    : velocity(statusMsg->velocity), control(statusMsg->control)
{
}

carla_msgs::CarlaEgoVehicleStatus VehicleStatus::getMessage() const noexcept
{
    carla_msgs::CarlaEgoVehicleStatus egoVehicleStatus;

    egoVehicleStatus.velocity = velocity;
    egoVehicleStatus.control = control.getMessage();

    return egoVehicleStatus;
}
