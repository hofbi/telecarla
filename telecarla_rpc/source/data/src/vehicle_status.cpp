#include "vehicle_status.h"

using namespace lmt::data;

VehicleStatus::VehicleStatus(const carla_msgs::CarlaEgoVehicleStatusConstPtr& statusMsg) noexcept
    : velocity(statusMsg->velocity), control(statusMsg->control)
{
}

carla_msgs::CarlaEgoVehicleStatus VehicleStatus::toROSMessage() const noexcept
{
    carla_msgs::CarlaEgoVehicleStatus egoVehicleStatus;

    egoVehicleStatus.velocity = velocity;
    egoVehicleStatus.control = control.toROSMessage();

    return egoVehicleStatus;
}
