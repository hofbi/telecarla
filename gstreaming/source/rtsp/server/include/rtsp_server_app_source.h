#ifndef GSTREAMING_RTSP_SERVER_APP_SOURCE_H
#define GSTREAMING_RTSP_SERVER_APP_SOURCE_H

#include <mutex>

#include <gst/gstcaps.h>
#include <gst/gstelement.h>
#include <sensor_msgs/Image.h>

namespace lmt
{
namespace rtsp
{
namespace server
{
class RTSPServerAppSource
{
  public:
    explicit RTSPServerAppSource(std::string mountName) noexcept;

    void setVideoData(const sensor_msgs::ImageConstPtr& msg);
    void updateSpatioTemporalResolution(uint8_t fps, uint8_t spatialScale) noexcept;
    void bufferNewData(GstElement* appSrc);

    const std::string& getName() const noexcept;

  private:
    sensor_msgs::ImageConstPtr getScaledImagePtr(const sensor_msgs::ImageConstPtr& msg) const;

  private:
    GstClockTime timestamp_{0};
    GstCaps* caps_{nullptr};
    sensor_msgs::ImageConstPtr imageMsg_;

    std::string name_;

    uint8_t fps_{0};
    uint8_t spatialScale_{100};
    bool forceKeyFrame_{false};

    std::mutex mutexLock_;
};
}  // namespace server
}  // namespace rtsp
}  // namespace lmt

#endif  // GSTREAMING_RTSP_SERVER_APP_SOURCE_H
