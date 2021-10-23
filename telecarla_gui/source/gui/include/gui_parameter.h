#ifndef TELECARLA_GUI_GUI_PARAMETER_H
#define TELECARLA_GUI_GUI_PARAMETER_H

#include <map>
#include <optional>
#include <vector>

#include <SDL2/SDL_rect.h>

namespace lmt::gui
{
class GuiParameter
{
  public:
    using TilesRow = std::vector<SDL_Rect>;

    explicit GuiParameter(const std::string_view& sensorJsonPath);

    [[nodiscard]] const std::map<std::string, SDL_Rect>& getCameraParameter() const;

    [[nodiscard]] int getWindowWidth() const noexcept;
    [[nodiscard]] int getWindowHeight() const noexcept;

    [[nodiscard]] const std::optional<SDL_Rect>& getVehicleStatusParameters() const noexcept;
    [[nodiscard]] const std::optional<SDL_Rect>& getStaticTextParameters() const noexcept;

  private:
    int windowWidth_{0};
    int windowHeight_{0};
    std::map<std::string, SDL_Rect> cameraParameter_;
    std::optional<SDL_Rect> vehicleStatusParameter_;
    std::optional<SDL_Rect> staticTextParameter_;
};

int getMaxHeight(const GuiParameter::TilesRow& tilesRow) noexcept;
}  // namespace lmt::gui

#endif  // TELECARLA_GUI_GUI_PARAMETER_H
