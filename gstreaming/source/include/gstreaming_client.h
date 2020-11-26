#ifndef _LMT_CLIENT_ROS_HH_
#define _LMT_CLIENT_ROS_HH_

#include <thread>

#include <image_transport/publisher.h>

#include "gst_lifecycle.h"
#include "rtsp_client.h"

namespace lmt
{
class GStreamingClient
{
  public:
    GStreamingClient(ros::NodeHandle& nh, ros::NodeHandle& pnh, int argc, char* argv[]);

  private:
    void start(const std::string& ip, int port, const std::string& name);
    void callbackImage(char* data, int size, int width, int height);
    void thrGstreamer();

  private:
    common::GstLifecycle gstLifecycle_;
    GMainLoop* loop_{nullptr};
    std::thread threadGstreamer_;
    std::unique_ptr<rtsp::client::RTSPClient> rtspClient_{nullptr};
    image_transport::Publisher pub_;
};

}  // namespace lmt

#endif /* _LMT_CLIENT_ROS_HH_ */
