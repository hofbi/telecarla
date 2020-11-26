#ifndef _LMT_SERVER_ROS_HH_
#define _LMT_SERVER_ROS_HH_

#include <thread>

#include <dynamic_reconfigure/server.h>
#include <ros/subscriber.h>

#include "gst_lifecycle.h"
#include "rtsp_server.h"

namespace lmt
{
class GStreamingServer
{
  public:
    GStreamingServer(ros::NodeHandle& nh, ros::NodeHandle& pnh, int argc, char* argv[]);

  private:
    void startStreaming();
    void thrGstreamer();

  private:
    common::GstLifecycle gstLifecycle_;
    int serverPort_;
    std::string mountName_;
    std::string streamSource_;
    GMainLoop* loop_{nullptr};
    std::thread threadGstreamer_;
    std::unique_ptr<rtsp::server::RTSPServer> rtspServer_{nullptr};
    ros::Subscriber subCamera_;
    dynamic_reconfigure::Server<gstreaming::RateControlConfig> rateControl_;
};

}  // namespace lmt

#endif /* _LMT_SERVER_ROS_HH_ */
