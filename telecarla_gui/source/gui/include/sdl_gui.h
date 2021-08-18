#ifndef TELECARLA_GUI_SDL_GUI_H
#define TELECARLA_GUI_SDL_GUI_H

#include <memory>

#include <SDL2/SDL_rect.h>
#include <SDL2/SDL_render.h>
#include <SDL2/SDL_video.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <sensor_msgs/Image.h>

namespace cv
{
class Mat;
}
namespace lmt::gui
{
class GuiParameter;

class SDL_GUI
{
  public:
    using TextLines = std::vector<std::pair<std::string_view, std::string_view>>;

    SDL_GUI() = default;

    explicit SDL_GUI(const GuiParameter& guiParameter);

    void show();

    void renderImage(const SDL_Rect& pos, const sensor_msgs::ImageConstPtr& msg, int imageFrequency) const;
    void renderStaticText(const SDL_Rect& pos, const TextLines& textLines) const;

  private:
    void renderCvMat(const SDL_Rect& pos, const cv::Mat& image) const;

  private:
    std::unique_ptr<SDL_Window, decltype(&SDL_DestroyWindow)> window_{nullptr, SDL_DestroyWindow};
    std::unique_ptr<SDL_Renderer, decltype(&SDL_DestroyRenderer)> renderer_{nullptr, SDL_DestroyRenderer};
    std::unique_ptr<SDL_Texture, decltype(&SDL_DestroyTexture)> texture_{nullptr, SDL_DestroyTexture};

    std::optional<SDL_Rect> staticTextParameters_;
};
}  // namespace lmt::gui

#endif  // TELECARLA_GUI_SDL_GUI_H
