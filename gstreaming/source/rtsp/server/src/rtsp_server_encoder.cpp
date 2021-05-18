#include "rtsp_server_encoder.h"

#include <gst/gstinfo.h>

using namespace lmt::rtsp::server;

RTSPServerEncoder::RTSPServerEncoder(std::string name, PadProbeCallback encoderProbeCallback) noexcept
    : name_(std::move(name)), encoderProbeCallback_(std::make_unique<PadProbeCallback>(std::move(encoderProbeCallback)))
{
}

void RTSPServerEncoder::configureEncoderElement(std::unique_ptr<GstElement> encoderElement) noexcept
{
    if (encoderElement == nullptr)
    {
        GST_ERROR("Could not get encoder with name %s from pipeline", name_.c_str());
        return;
    }

    encoderElement_ = std::move(encoderElement);
    auto x264EncPad = std::unique_ptr<GstPad, decltype(&gst_object_unref)>(
        gst_element_get_static_pad(encoderElement_.get(), "src"), gst_object_unref);
    auto wrappedProbeCallback = [](GstPad* pad, GstPadProbeInfo* info, void* userdata) {
        auto& callback = *static_cast<RTSPServerEncoder::PadProbeCallback*>(userdata);
        return callback(pad, info);
    };
    gst_pad_add_probe(
        x264EncPad.get(), GST_PAD_PROBE_TYPE_BUFFER, wrappedProbeCallback, encoderProbeCallback_.get(), nullptr);
}

const std::string& RTSPServerEncoder::getName() const noexcept
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
