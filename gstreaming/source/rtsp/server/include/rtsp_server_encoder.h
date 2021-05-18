#ifndef GSTREAMING_RTSP_SERVER_ENCODER_H
#define GSTREAMING_RTSP_SERVER_ENCODER_H

#include <functional>
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
    using PadProbeCallback = std::function<GstPadProbeReturn(GstPad*, GstPadProbeInfo*)>;

    RTSPServerEncoder(std::string name, PadProbeCallback encoderProbeCallback) noexcept;

    void configureEncoderElement(std::unique_ptr<GstElement> encoderElement) noexcept;
    void updateRateControlParameter(int bitrate);

    const std::string& getName() const noexcept;

  private:
    std::string name_;
    std::unique_ptr<PadProbeCallback> encoderProbeCallback_{nullptr};
    std::unique_ptr<GstElement> encoderElement_{nullptr};
};
}  // namespace server
}  // namespace rtsp
}  // namespace lmt

#endif  // GSTREAMING_RTSP_SERVER_ENCODER_H
