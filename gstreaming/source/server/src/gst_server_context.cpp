#include "gst_server_context.h"

using namespace lmt::server;

GstServerContext::GstServerContext(const std::string& mountName,
                                   const std::string& encoderName,
                                   const GstServerEncoder::PadProbeCallback& encoderProbeCallback) noexcept
    : app(std::make_unique<GstServerAppSource>(mountName)),
      encoder(std::make_unique<GstServerEncoder>(encoderName, encoderProbeCallback))
{
}
