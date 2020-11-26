#include "rtsp_server.h"

#include "gst_rtsp_server_util.h"

using namespace lmt::rtsp::server;
using namespace lmt::rtsp::common;

RTSPState RTSPServer::start(int serverPort, const std::string& src)
{
    if (state_ == RTSPState::stopped)
    {
        state_ = RTSPState::starting;

        server_.reset(gst_rtsp_server_new());
        gst_rtsp_server_set_service(server_.get(), std::to_string(serverPort).c_str());

        auto mounts = std::unique_ptr<GstRTSPMountPoints, decltype(&g_object_unref)>(
            gst_rtsp_server_get_mount_points(server_.get()), g_object_unref);

        std::stringstream command;
        command << src << " name=" << context_->app->getName()
                << " is-live=true ! videorate !  videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast "
                   "sliced-threads=true byte-stream=true threads=1 key-int-max=15 intra-refresh=true name="
                << context_->encoder->getName() << " ! h264parse ! rtph264pay pt=96 name=pay0";

        GstRTSPMediaFactory* factory = gst_rtsp_media_factory_new();

        gst_rtsp_media_factory_set_launch(factory, command.str().c_str());
        //        gst_rtsp_media_factory_set_protocols(factory, GST_RTSP_LOWER_TRANS_TCP);
        //        gst_rtsp_media_factory_set_protocols(factory, GST_RTSP_LOWER_TRANS_UDP);

        std::string path = "/" + context_->app->getName();
        gst_rtsp_mount_points_add_factory(mounts.get(), path.c_str(), factory);

        g_signal_connect(factory, "media-configure", (GCallback)mediaConfigure, context_.get());

        handle_ = gst_rtsp_server_attach(server_.get(), nullptr);

        state_ = RTSPState::started;
    }
    return state_;
}

void RTSPServer::stop()
{
    if (state_ == RTSPState::started)
    {
        state_ = RTSPState::stopping;

        if (handle_ != -1)
        {
            g_source_remove(handle_);
        }

        handle_ = -1;
        state_ = RTSPState::stopped;
    }
}

RTSPServer::RTSPServer(const std::string& mountName)
    : context_(std::make_unique<RTSPServerContext>(mountName, "x264encoder"))
{
    const auto videoWidthInPixels{640};
    const auto videoHeightInPixels{480};
    context_->app->setVideoData(getDefaultImage(videoWidthInPixels, videoHeightInPixels, 0));
}

RTSPServer::~RTSPServer()
{
    stop();
}

void RTSPServer::rateControlCallback(gstreaming::RateControlConfig& config, uint32_t /*level*/)
{
    context_->encoder->updateRateControlParameter(config.bitrate);
    context_->app->updateSpatioTemporalResolution(config.fps, config.spatial_scale);
}

void RTSPServer::cameraImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    context_->app->setVideoData(msg);
}
