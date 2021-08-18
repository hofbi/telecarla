#ifndef _LMT_RTSP_SERVER_HH_
#define _LMT_RTSP_SERVER_HH_

#include <gst/rtsp-server/rtsp-media-factory.h>
#include <gstreaming/RateControlConfig.h>

#include "gst_server_context.h"
#include "pipeline_state.h"

namespace lmt
{
namespace server
{
class RTSPServer
{
  public:
    RTSPServer(const std::string& mountName, const GstServerEncoder::PadProbeCallback& encoderProbeCallback);
    ~RTSPServer();

    RTSPServer(const RTSPServer& rhs) = delete;
    RTSPServer(RTSPServer&& rhs) = delete;

    RTSPServer& operator=(const RTSPServer& rhs) = delete;
    RTSPServer& operator=(RTSPServer&& rhs) = delete;

    common::PipelineState start(int serverPort, const std::string& src);
    void stop();

    void rateControlCallback(gstreaming::RateControlConfig& config, uint32_t level);
    void cameraImageCallback(const sensor_msgs::ImageConstPtr& msg);

  private:
    std::unique_ptr<GstRTSPServer, decltype(&g_object_unref)> server_{nullptr, g_object_unref};
    int handle_{-1};
    common::PipelineState state_{common::PipelineState::stopped};
    std::unique_ptr<GstServerContext> context_{nullptr};
};
}  // namespace server
}  // namespace lmt

#endif /* _LMT_RTSP_SERVER_HH_ */
