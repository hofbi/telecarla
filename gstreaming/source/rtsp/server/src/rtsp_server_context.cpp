#include "rtsp_server_context.h"

using namespace lmt::rtsp::server;

RTSPServerContext::RTSPServerContext(const std::string& mountName, const std::string& /*encoderName*/) noexcept
    : app(std::make_unique<RTPSServerAppSource>(mountName)), encoder(std::make_unique<RTSPServerEncoder>("x264encoder"))
{
}
