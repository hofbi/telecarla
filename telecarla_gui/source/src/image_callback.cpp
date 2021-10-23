#include "image_callback.h"

#include <numeric>

using namespace lmt;

namespace
{
constexpr auto timeDiffHistoryBufferSize = 20U;
}  // namespace

ImageCallback::ImageCallback(const SDL_Rect& position, RenderCallback renderCallback) noexcept
    : position_(position),
      renderCallback_(std::move(renderCallback)),
      callbackTimeDiffHistory_(timeDiffHistoryBufferSize, 0.0)
{
}

void ImageCallback::operator()(const sensor_msgs::ImageConstPtr& msg)
{
    const auto diff = ros::Time::now() - lastCallbackTime_;
    lastCallbackTime_ = ros::Time::now();

    callbackTimeDiffHistory_.push_back(diff.toSec());

    const auto mean = std::accumulate(callbackTimeDiffHistory_.begin(), callbackTimeDiffHistory_.end(), 0.0) /
                      static_cast<double>(callbackTimeDiffHistory_.size());

    renderCallback_(position_, msg, static_cast<int>(1 / mean));
}
