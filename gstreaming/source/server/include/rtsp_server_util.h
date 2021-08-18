#ifndef GSTREAMING_RTSP_SERVER_UTIL_H
#define GSTREAMING_RTSP_SERVER_UTIL_H

#include <gst/rtsp-server/rtsp-media-factory.h>

#include "gst_server_context.h"

namespace lmt
{
namespace server
{
void needData(GstElement* appSrc, guint unused, GstServerContext* context);
void mediaConfigure(GstRTSPMediaFactory* factory, GstRTSPMedia* media, GstServerContext* context);
}  // namespace server
}  // namespace lmt

#endif  // GSTREAMING_RTSP_SERVER_UTIL_H
