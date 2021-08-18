#include "rtsp_server_util.h"

#include <glib.h>
#include <ros/console.h>
#include <sensor_msgs/image_encodings.h>

#include "gst_server_context.h"

namespace lmt
{
namespace server
{
void needData(GstElement* appSrc, guint /*unused*/, GstServerContext* context)
{
    context->app->bufferNewData(appSrc);
}

void mediaConfigure(GstRTSPMediaFactory* /*factory*/, GstRTSPMedia* media, GstServerContext* context)
{
    auto element =
        std::unique_ptr<GstElement, decltype(&gst_object_unref)>(gst_rtsp_media_get_element(media), gst_object_unref);
    GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(element.get()), GST_DEBUG_GRAPH_SHOW_ALL, "pipeline-rtsp-server");
    auto appSrc = std::unique_ptr<GstElement, decltype(&gst_object_unref)>(
        gst_bin_get_by_name_recurse_up(GST_BIN(element.get()), context->app->getName().c_str()), gst_object_unref);

    context->encoder->configureEncoderElement(std::unique_ptr<GstElement>(
        gst_bin_get_by_name_recurse_up(GST_BIN(element.get()), context->encoder->getName().c_str())));

    gst_util_set_object_arg(G_OBJECT(appSrc.get()), "format", "time");
    g_object_set_data_full(G_OBJECT(media), "my-extra-data", context->app.get(), (GDestroyNotify)g_free);
    g_signal_connect(appSrc.get(), "need-data", (GCallback)needData, context);

    ROS_INFO("Media configured done");
}
}  // namespace server
}  // namespace lmt
