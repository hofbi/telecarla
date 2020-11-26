#ifndef GSTREAMING_RTSP_SERVER_ENCODER_H
#define GSTREAMING_RTSP_SERVER_ENCODER_H

#include <memory>
#include <string>

#include <gst/gstelement.h>

namespace lmt
{
namespace rtsp
{
namespace server
{
class RTSPServerEncoder
{
  public:
    explicit RTSPServerEncoder(std::string name);

    void setEncoderElement(std::unique_ptr<GstElement> encoderElement);
    void updateRateControlParameter(int bitrate);

    const std::string& getName() const;

  private:
    std::string name_;
    std::unique_ptr<GstElement> encoderElement_{nullptr};
};
}  // namespace server
}  // namespace rtsp
}  // namespace lmt

#endif  // GSTREAMING_RTSP_SERVER_ENCODER_H
