#ifndef TELECARLA_GUI_SDL_EVENT_PARSER_H
#define TELECARLA_GUI_SDL_EVENT_PARSER_H

#include <functional>
#include <limits>

#include <SDL2/SDL_joystick.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <telecarla_msgs/TeleopMode.h>

#include "joystick_parameter.h"
#include "linear_function.h"
#include "steer_cache.h"

namespace lmt::gui
{
class SDL_EventParser
{
  public:
    using TeleopModeCallback = std::function<void(const telecarla_msgs::TeleopMode&)>;

    explicit SDL_EventParser(TeleopModeCallback teleopModeCallback);

    void pollEvents();

    [[nodiscard]] const carla_msgs::CarlaEgoVehicleControl& getCarlaEgoVehicleControl() const noexcept;
    [[nodiscard]] double getSteerCache() const noexcept;

  private:
    void switchTeleopMode(uint8_t mode);

  private:
    carla_msgs::CarlaEgoVehicleControl carlaEgoVehicleControl_;
    TeleopModeCallback teleopModeCallback_;
    JoystickParameter joystickParameter_{};
    util::SteerCache steerCache_{};
    std::unique_ptr<SDL_Joystick, decltype(&SDL_JoystickClose)> joystick_{nullptr, SDL_JoystickClose};

  private:
    static constexpr util::LinearFunction wheelFunction_{std::make_pair(std::numeric_limits<int16_t>::max(), 1.0),
                                                         std::make_pair(std::numeric_limits<int16_t>::min(), -1.0)};
    static constexpr util::LinearFunction pedalFunction_{std::make_pair(std::numeric_limits<int16_t>::max(), 0.0),
                                                         std::make_pair(std::numeric_limits<int16_t>::min(), 1.0)};
};
}  // namespace lmt::gui

#endif  // TELECARLA_GUI_SDL_EVENT_PARSER_H
