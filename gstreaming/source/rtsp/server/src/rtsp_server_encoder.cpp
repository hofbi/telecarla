#include "rtsp_server_encoder.h"

#include <gst/gstinfo.h>

using namespace lmt::rtsp::server;

RTSPServerEncoder::RTSPServerEncoder(std::string name) : name_(std::move(name)) {}

void RTSPServerEncoder::setEncoderElement(std::unique_ptr<GstElement> encoderElement)
{
    if (encoderElement == nullptr)
    {
        GST_ERROR("Could not get encoder with name %s from pipeline", name_.c_str());
    }
    else
    {
        encoderElement_ = std::move(encoderElement);
    }
}

const std::string& RTSPServerEncoder::getName() const
{
    return name_;
}

void RTSPServerEncoder::updateRateControlParameter(int bitrate)
{
    if (encoderElement_ != nullptr)
    {
        g_object_set(G_OBJECT(encoderElement_.get()), "bitrate", bitrate, nullptr);
    }
}
