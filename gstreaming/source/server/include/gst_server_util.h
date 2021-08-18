#ifndef GSTREAMING_GST_SERVER_UTIL_H
#define GSTREAMING_GST_SERVER_UTIL_H

#include <gst/gstcaps.h>
#include <sensor_msgs/Image.h>

namespace lmt
{
namespace server
{
sensor_msgs::ImageConstPtr getDefaultImage(int width, int height) noexcept;

GstCaps* gstCapsFromImage(const sensor_msgs::Image::ConstPtr& msg, int framerate);
}  // namespace server
}  // namespace lmt

#endif  // GSTREAMING_GST_SERVER_UTIL_H
