#include "rtsp_server_app_source.h"

#include <cv_bridge/cv_bridge.h>
#include <gst/base/gstbasetransform.h>
#include <gst/video/video.h>

#include "gst_rtsp_server_util.h"

using namespace lmt::rtsp::server;

RTSPServerAppSource::RTSPServerAppSource(std::string mountName) noexcept : name_(std::move(mountName)) {}

void RTSPServerAppSource::setVideoData(const sensor_msgs::ImageConstPtr& msg)
{
    mutexLock_.lock();

    imageMsg_ = getScaledImagePtr(msg);
    caps_ = gstCapsFromImage(imageMsg_, fps_);

    mutexLock_.unlock();
}

sensor_msgs::ImageConstPtr RTSPServerAppSource::getScaledImagePtr(const sensor_msgs::ImageConstPtr& msg) const
{
    const auto defaultSpatialScale{100};
    if (spatialScale_ != defaultSpatialScale)
    {
        cv::Mat image;
        const auto newWidth = spatialScale_ * msg->width / defaultSpatialScale;
        const auto newHeight = spatialScale_ * msg->height / defaultSpatialScale;

        cv::resize(cv_bridge::toCvShare(msg)->image, image, cv::Size(newWidth, newHeight));

        cv_bridge::CvImage newImage(msg->header, msg->encoding, image);
        return newImage.toImageMsg();
    }
    return msg;
}

const std::string& RTSPServerAppSource::getName() const noexcept
{
    return name_;
}

void RTSPServerAppSource::bufferNewData(GstElement* appSrc)
{
    mutexLock_.lock();
    g_object_set(G_OBJECT(appSrc), "caps", caps_, nullptr);

    auto* buffer = gst_buffer_new_allocate(nullptr, imageMsg_->data.size(), nullptr);
    gst_buffer_fill(buffer, 0, imageMsg_->data.data(), imageMsg_->data.size());
    mutexLock_.unlock();

    if (forceKeyFrame_)
    {
        auto* event = gst_video_event_new_downstream_force_key_unit(timestamp_, 0, 0, TRUE, 1);
        auto* pad = gst_element_get_static_pad(appSrc, "src");
        gst_pad_push_event(pad, event);

        forceKeyFrame_ = false;
    }

    GST_BUFFER_FLAG_SET(buffer, GST_BUFFER_FLAG_LIVE);
    GST_BUFFER_PTS(buffer) = timestamp_;
    const auto denominator{20};
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, denominator);
    timestamp_ += GST_BUFFER_DURATION(buffer);

    GST_BUFFER_DTS(buffer) = GST_BUFFER_PTS(buffer);

    GstFlowReturn ret;
    g_signal_emit_by_name(appSrc, "push-buffer", buffer, &ret);
}

void RTSPServerAppSource::updateSpatioTemporalResolution(uint8_t fps, uint8_t spatialScale) noexcept
{
    fps_ = fps;
    if (spatialScale_ != spatialScale)
    {
        spatialScale_ = spatialScale;
        forceKeyFrame_ = true;
    }
}
