#include "status_callback.h"

using namespace lmt;

StatusCallback::StatusCallback(const SDL_Rect& position, StatusCallback::RenderCallback renderCallback) noexcept
    : position_(position), renderCallback_(std::move(renderCallback))
{
}

void StatusCallback::operator()(const carla_msgs::CarlaEgoVehicleStatusConstPtr& vehicleStatusMessage)
{
    const auto factor_mps_to_kph{3.6};
    renderCallback_(position_,
                    {{"Vehicle Status:", ""},
                     {},
                     {"  Velocity [m/s]  ", std::to_string(vehicleStatusMessage->velocity)},
                     {"  Velocity [km/h] ", std::to_string(vehicleStatusMessage->velocity * factor_mps_to_kph)},
                     {"  Reverse         ", std::to_string(vehicleStatusMessage->control.reverse)}});
}
