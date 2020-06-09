#include "status_callback.h"

using namespace lmt;

StatusCallback::StatusCallback(const SDL_Rect& position, StatusCallback::RenderCallback renderCallback) noexcept
    : position_(position), renderCallback_(std::move(renderCallback))
{
}

void StatusCallback::operator()(const carla_msgs::CarlaEgoVehicleStatusConstPtr& vehicleStatusMessage)
{
    renderCallback_(position_,
                    {{"Vehicle Status:", ""},
                     {},
                     {"  Velocity [m/s]  ", std::to_string(vehicleStatusMessage->velocity)},
                     {"  Velocity [km/h] ", std::to_string(vehicleStatusMessage->velocity * 3.6)},
                     {"  Reverse         ", std::to_string(vehicleStatusMessage->control.reverse)}});
}
