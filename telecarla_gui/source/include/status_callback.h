#ifndef TELECARLA_GUI_STATUS_CALLBACK_H
#define TELECARLA_GUI_STATUS_CALLBACK_H

#include <functional>

#include <SDL2/SDL_rect.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>

namespace lmt
{
class StatusCallback
{
  public:
    using RenderCallback =
        std::function<void(const SDL_Rect& pos,
                           const std::vector<std::pair<std::string_view, std::string_view>>& textLines)>;

    StatusCallback(const SDL_Rect& position, RenderCallback renderCallback) noexcept;

    void operator()(const carla_msgs::CarlaEgoVehicleStatusConstPtr& vehicleStatusMessage);

  private:
    SDL_Rect position_;
    RenderCallback renderCallback_;
};
}  // namespace lmt

#endif  // TELECARLA_GUI_STATUS_CALLBACK_H
