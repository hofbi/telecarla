#ifndef GSTREAMING_RTSP_SERVER_CONTEXT_H
#define GSTREAMING_RTSP_SERVER_CONTEXT_H

#include "rtsp_server_app_source.h"
#include "rtsp_server_encoder.h"

namespace lmt
{
namespace rtsp
{
namespace server
{
struct RTSPServerContext
{
    RTSPServerContext(const std::string& mountName, const std::string& encoderName) noexcept;

    std::unique_ptr<RTPSServerAppSource> app{nullptr};
    std::unique_ptr<RTSPServerEncoder> encoder{nullptr};
};
}  // namespace server
}  // namespace rtsp
}  // namespace lmt

#endif  // GSTREAMING_RTSP_SERVER_CONTEXT_H
