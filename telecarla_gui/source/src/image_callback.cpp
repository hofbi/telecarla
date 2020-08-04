#include "image_callback.h"

#include <numeric>

using namespace lmt;

namespace
{
const auto timeDiffHistoryBufferSize{20U};
}  // namespace

ImageCallback::ImageCallback(const SDL_Rect& position, RenderCallback renderCallback) noexcept
    : position_(position),
      renderCallback_(std::move(renderCallback)),
      callbackTimeDiffHistory_(timeDiffHistoryBufferSize, 0.0)
{
}

void ImageCallback::operator()(const sensor_msgs::ImageConstPtr& msg)
{
    ros::Duration diff = ros::Time::now() - lastCallbackTime_;
    lastCallbackTime_ = ros::Time::now();

    callbackTimeDiffHistory_.push_back(diff.toSec());

    double mean = std::accumulate(callbackTimeDiffHistory_.begin(), callbackTimeDiffHistory_.end(), 0.0) /
                  callbackTimeDiffHistory_.size();

    renderCallback_(position_, msg, 1 / mean);
}
