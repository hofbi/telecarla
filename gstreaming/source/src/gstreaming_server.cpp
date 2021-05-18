#include "gstreaming_server.h"

#include <utility>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <telecarla_msgs/FrameStatistics.h>

using namespace lmt;
using namespace lmt::rtsp::server;

GStreamingServer::GStreamingServer(ros::NodeHandle& nh,
                                   ros::NodeHandle& pnh,
                                   int argc,
                                   char* argv[])  // NOLINT(modernize-avoid-c-arrays)
    : gstLifecycle_(argc, argv),
      serverPort_(pnh.param("port", 8551)),  // NOLINT(readability-magic-numbers)
      mountName_(pnh.param("mount", std::string("mainstream"))),
      streamSource_(pnh.param("stream_source", std::string("appsrc"))),
      loop_(g_main_loop_new(nullptr, false)),
      threadGstreamer_(std::thread(&GStreamingServer::thrGstreamer, this)),
      rtspServer_(std::make_unique<RTSPServer>(mountName_, [this](auto&&... args) {
          return frameStatCallback(std::forward<decltype(args)>(args)...);
      }))
{
    std::string inTopic;
    if (!pnh.getParam("in_topic", inTopic))
    {
        ROS_ERROR("No input topic provided. Set argument in_topic");
        return;
    }
    const auto outStatTopic = pnh.param("stat_topic", std::string("/camera/stats"));

    subCamera_ = nh.subscribe(inTopic, 1, &RTSPServer::cameraImageCallback, rtspServer_.get());
    pubStat_ = nh.advertise<telecarla_msgs::FrameStatistics>(pnh.getNamespace() + outStatTopic, 1);
    ROS_INFO_STREAM("Subscribed to image topic " << inTopic);
    ROS_INFO_STREAM("Provide topic " << outStatTopic);
    rateControl_.setCallback(
        [this](auto&&... args) { rtspServer_->rateControlCallback(std::forward<decltype(args)>(args)...); });

    startStreaming();
}

void GStreamingServer::startStreaming()
{
    if (rtsp::common::RTSPState::started == rtspServer_->start(serverPort_, streamSource_))
    {
        ROS_INFO_STREAM("RTSP SERVER started and ready at rtsp://127.0.0.1:" << serverPort_ << "/" << mountName_);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to start RTSP Server at rtsp://127.0.0.1:" << serverPort_ << "/" << mountName_);
    }
}

void GStreamingServer::thrGstreamer()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // NOLINT(readability-magic-numbers)
    g_main_loop_run(loop_);
    g_main_loop_quit(loop_);
    g_main_loop_unref(loop_);
}

GstPadProbeReturn GStreamingServer::frameStatCallback(GstPad* /*pad*/, GstPadProbeInfo* info)
{
    auto* buffer = gst_pad_probe_info_get_buffer(info);  // no need to unref
    telecarla_msgs::FrameStatistics msg;
    msg.frameType = gst_buffer_has_flags(buffer, GST_BUFFER_FLAG_DELTA_UNIT);
    msg.packetSize = gst_buffer_get_size(buffer);
    pubStat_.publish(msg);
    return GST_PAD_PROBE_OK;
}
