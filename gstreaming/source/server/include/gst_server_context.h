#ifndef GSTREAMING_GST_SERVER_CONTEXT_H
#define GSTREAMING_GST_SERVER_CONTEXT_H

#include "gst_server_app_source.h"
#include "gst_server_encoder.h"

namespace lmt
{
namespace server
{
struct GstServerContext
{
    GstServerContext(const std::string& mountName,
                     const std::string& encoderName,
                     const GstServerEncoder::PadProbeCallback& encoderProbeCallback) noexcept;

    std::unique_ptr<GstServerAppSource> app{nullptr};
    std::unique_ptr<GstServerEncoder> encoder{nullptr};
};
}  // namespace server
}  // namespace lmt

#endif  // GSTREAMING_GST_SERVER_CONTEXT_H
