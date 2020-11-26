#ifndef TELECARLA_GUI_SDL_WHEEL_CONTROLLER_H
#define TELECARLA_GUI_SDL_WHEEL_CONTROLLER_H

#include <memory>

#include <SDL2/SDL_haptic.h>

namespace lmt::gui
{
class SDL_EventParser;

class SDL_WheelController
{
  public:
    explicit SDL_WheelController(const SDL_EventParser& parser);

    void setTargetWheelPosition(double targetPosition);
    [[nodiscard]] bool isAvailable() const noexcept;

  private:
    std::unique_ptr<SDL_Haptic, decltype(&SDL_HapticClose)> wheel_{nullptr, SDL_HapticClose};
    const SDL_EventParser& parser_;
};

SDL_HapticEffect makeSteeringEffect(int direction);
SDL_HapticEffect makeLeftSteeringEffect();
SDL_HapticEffect makeRightSteeringEffect();
}  // namespace lmt::gui
#endif  // TELECARLA_GUI_SDL_WHEEL_CONTROLLER_H
