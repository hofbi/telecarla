#include "rtsp_server_context.h"

using namespace lmt::rtsp::server;

RTSPServerContext::RTSPServerContext(const std::string& mountName,
                                     const std::string& encoderName,
                                     const RTSPServerEncoder::PadProbeCallback& encoderProbeCallback) noexcept
    : app(std::make_unique<RTSPServerAppSource>(mountName)),
      encoder(std::make_unique<RTSPServerEncoder>(encoderName, encoderProbeCallback))
{
}
