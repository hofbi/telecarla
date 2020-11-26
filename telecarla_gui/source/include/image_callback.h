#ifndef TELECARLA_GUI_IMAGE_CALLBACK_H
#define TELECARLA_GUI_IMAGE_CALLBACK_H

#include <functional>
#include <memory>

#include <SDL2/SDL_rect.h>
#include <boost/circular_buffer.hpp>
#include <sensor_msgs/Image.h>

class SDL_Texture;

namespace lmt
{
class ImageCallback
{
  public:
    using RenderCallback =
        std::function<void(const SDL_Rect& pos, const sensor_msgs::ImageConstPtr& msg, int imageFrequency)>;

    ImageCallback(const SDL_Rect& position, RenderCallback renderCallback) noexcept;

    void operator()(const sensor_msgs::ImageConstPtr& msg);

  private:
    SDL_Rect position_;
    RenderCallback renderCallback_;
    ros::Time lastCallbackTime_{ros::Time::now()};
    boost::circular_buffer<double> callbackTimeDiffHistory_;
};
}  // namespace lmt

#endif  // TELECARLA_GUI_IMAGE_CALLBACK_H
