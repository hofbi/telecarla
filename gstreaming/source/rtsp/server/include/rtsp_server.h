#ifndef _LMT_RTSP_SERVER_HH_
#define _LMT_RTSP_SERVER_HH_

#include <gst/rtsp-server/rtsp-media-factory.h>
#include <gstreaming/RateControlConfig.h>

#include "rtsp_server_context.h"
#include "rtsp_state.h"

namespace lmt
{
namespace rtsp
{
namespace server
{
class RTSPServer
{
  public:
    explicit RTSPServer(const std::string& mountName);
    ~RTSPServer();

    RTSPServer(const RTSPServer& rhs) = delete;
    RTSPServer(RTSPServer&& rhs) = delete;

    RTSPServer& operator=(const RTSPServer& rhs) = delete;
    RTSPServer& operator=(RTSPServer&& rhs) = delete;

    common::RTSPState start(int serverPort, const std::string& src);
    void stop();

    void rateControlCallback(gstreaming::RateControlConfig& config, uint32_t level);
    void cameraImageCallback(const sensor_msgs::ImageConstPtr& msg);

  private:
    std::unique_ptr<GstRTSPServer, decltype(&g_object_unref)> server_{nullptr, g_object_unref};
    int handle_{-1};
    common::RTSPState state_{common::RTSPState::stopped};
    std::unique_ptr<RTSPServerContext> context_{nullptr};
};
}  // namespace server
}  // namespace rtsp
}  // namespace lmt

#endif /* _LMT_VIDEO_STREAM_ROS_HH_ */
