#include "gstreaming_client.h"

#include <utility>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace lmt;

GStreamingClient::GStreamingClient(ros::NodeHandle& nh,
                                   ros::NodeHandle& pnh,
                                   int argc,
                                   char* argv[])  // NOLINT(modernize-avoid-c-arrays)
    : gstLifecycle_(argc, argv),
      loop_(g_main_loop_new(nullptr, false)),
      threadGstreamer_(&GStreamingClient::thrGstreamer, this),
      rtspClient_(std::make_unique<rtsp::client::RTSPClient>(
          [this](auto&&... args) { callbackImage(std::forward<decltype(args)>(args)...); }))
{
    image_transport::ImageTransport it(nh);
    const auto outTopic = pnh.param("out_topic", std::string("/camera/rgb/image_color"));
    pub_ = it.advertise(pnh.getNamespace() + outTopic, 1);
    ROS_INFO_STREAM("Provide topic " << pnh.getNamespace() << outTopic);

    const auto ip = pnh.param("host", std::string("127.0.0.1"));
    const auto port = pnh.param("port", 8551);
    const auto name = pnh.param("mount", std::string("mainstream"));

    start(ip, port, name);
}

void GStreamingClient::callbackImage(char* data, int /*size*/, int width, int height)
{
    const auto image = cv::Mat(cv::Size(width, height), CV_8UC3, data, cv::Mat::AUTO_STEP);

    if (!image.empty())
    {
        pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "rgb8", image).toImageMsg());
    }
}

void GStreamingClient::thrGstreamer()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // NOLINT(readability-magic-numbers)
    g_main_loop_run(loop_);
    g_main_loop_quit(loop_);
    g_main_loop_unref(loop_);
}

void GStreamingClient::start(const std::string& ip, int port, const std::string& name)
{
    if (rtsp::common::RTSPState::started == rtspClient_->start(ip, port, name))
    {
        ROS_INFO_STREAM("RTSP Client connected to " << ip << ":" << port << "/" << name);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to connect RTSP Client to " << ip << ":" << port << "/" << name);
    }
}
