#include "gst_server_util.h"

#include <cv_bridge/cv_bridge.h>
#include <glib.h>
#include <gst/gstvalue.h>
#include <ros/console.h>

namespace lmt
{
namespace server
{
sensor_msgs::ImageConstPtr getDefaultImage(int width, int height) noexcept
{
    return cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, cv::Mat(height, width, CV_8UC3))
        .toImageMsg();
}

GstCaps* gstCapsFromImage(const sensor_msgs::Image::ConstPtr& msg, int framerate)
{
    // http://gstreamer.freedesktop.org/data/doc/gstreamer/head/pwg/html/section-types-definitions.html
    static const ros::M_string knownFormats = {{
        {sensor_msgs::image_encodings::RGB8, "RGB"},
        {sensor_msgs::image_encodings::RGB16, "RGB16"},
        {sensor_msgs::image_encodings::RGBA8, "RGBA"},
        {sensor_msgs::image_encodings::RGBA16, "RGBA16"},
        {sensor_msgs::image_encodings::BGR8, "BGR"},
        {sensor_msgs::image_encodings::BGR16, "BGR16"},
        {sensor_msgs::image_encodings::BGRA8, "BGRA"},
        {sensor_msgs::image_encodings::BGRA16, "BGRA16"},
        {sensor_msgs::image_encodings::MONO8, "GRAY8"},
        {sensor_msgs::image_encodings::MONO16, "GRAY16_LE"},
    }};

    if (msg->is_bigendian)
    {
        ROS_ERROR("GST: big endian image format is not supported");
        return nullptr;
    }

    const auto format = knownFormats.find(msg->encoding);
    if (format == knownFormats.end())
    {
        ROS_ERROR("GST: image format '%s' unknown", msg->encoding.c_str());
        return nullptr;
    }

    return gst_caps_new_simple("video/x-raw",
                               "format",
                               G_TYPE_STRING,
                               format->second.c_str(),
                               "width",
                               G_TYPE_INT,
                               msg->width,
                               "height",
                               G_TYPE_INT,
                               msg->height,
                               "framerate",
                               GST_TYPE_FRACTION,
                               framerate,
                               1,
                               nullptr);
}
}  // namespace server
}  // namespace lmt
