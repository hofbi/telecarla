#ifndef GSTREAMING_GST_RTSP_SERVER_UTIL_H
#define GSTREAMING_GST_RTSP_SERVER_UTIL_H

#include <gst/rtsp-server/rtsp-media-factory.h>

#include "rtsp_server_context.h"

namespace lmt
{
namespace rtsp
{
namespace server
{
sensor_msgs::ImageConstPtr getDefaultImage(int width, int height, int frameCount);

GstCaps* gstCapsFromImage(const sensor_msgs::Image::ConstPtr& msg, int framerate);

void needData(GstElement* appSrc, guint unused, RTSPServerContext* context);
void mediaConfigure(GstRTSPMediaFactory* factory, GstRTSPMedia* media, RTSPServerContext* context);
}  // namespace server
}  // namespace rtsp
}  // namespace lmt

#endif  // GSTREAMING_GST_RTSP_SERVER_UTIL_H
