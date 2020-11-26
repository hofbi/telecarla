#include "rtsp_client.h"

#include <vector>

#include <glib.h>
#include <gst/gst.h>

using namespace lmt::rtsp::client;
using namespace lmt::rtsp::common;

GstFlowReturn RTSPClient::onNewSampleFromSink(GstAppSink* appSink, RTSPClient* data)
{
    GstSample* sample = gst_app_sink_pull_sample(appSink);
    GstCaps* caps = gst_sample_get_caps(sample);

    if (caps == nullptr)
    {
        GST_ERROR("Could not get image info from filter caps");
        return GST_FLOW_ERROR;
    }

    GstStructure* s = gst_caps_get_structure(caps, 0);
    int width = 0;
    int height = 0;
    if (!(gst_structure_get_int(s, "width", &width) && gst_structure_get_int(s, "height", &height)))
    {
        GST_ERROR("Could not get image width and height from filter caps");
        return GST_FLOW_ERROR;
    }

    GstBuffer* buffer = gst_sample_get_buffer(sample);
    GstMemory* mem = gst_buffer_get_all_memory(buffer);
    GstMapInfo info;
    gst_memory_map(mem, &info, GST_MAP_READ);

    if (data->callBack_)
    {
        data->callBack_(reinterpret_cast<char*>(info.data), info.size, width, height);
    }

    gst_sample_unref(sample);
    gst_memory_unmap(mem, &info);

    return GST_FLOW_OK;
}

RTSPState RTSPClient::start(const std::string& serverHost, int serverPort, const std::string& serverMount)
{
    switch (state_)
    {
        case RTSPState::stopped: {
            // rtspsrc default: protocols=tcp+udp-mcast+udp
            std::string cmd =
                "rtspsrc location=rtsp://" + serverHost + ":" + std::to_string(serverPort) + "/" + serverMount +
                " ! rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=RGB ! appsink name=" + serverMount +
                " sync=false";

            GError* error = nullptr;
            pipeline_ = gst_parse_launch(cmd.c_str(), &error);
            GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline_), GST_DEBUG_GRAPH_SHOW_ALL, "pipeline-rtsp-client");

            receiverAppSink_ = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline_), serverMount.c_str()));
            gst_app_sink_set_emit_signals(receiverAppSink_, true);
            g_signal_connect(receiverAppSink_, "new-sample", G_CALLBACK(onNewSampleFromSink), this);

            if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
            {
                GST_ERROR("Failed to set state to PLAYING\n");
                stop();
            }
            else
            {
                GST_INFO("Client is PLAYING\n");
                state_ = RTSPState::started;
            }
            break;
        }
        case RTSPState::paused: {
            resume();
            break;
        }
        default:
            break;
    }
    return state_;
}

void RTSPClient::resume()
{
    if (state_ == RTSPState::paused)
    {
        if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
        {
            GST_ERROR("Failed to set state to GST_STATE_PLAYING\n");
            stop();
        }
        else
        {
            GST_INFO("RTSP Client GST_STATE_PLAYING\n");
        }
    }
}

void RTSPClient::stop()
{
    if (state_ != RTSPState::stopped)
    {
        GST_INFO("Gstreamer client terminating...\n");
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        g_object_unref(pipeline_);
        state_ = RTSPState::stopped;
        GST_INFO("Gstreamer client terminated\n");
    }
}

RTSPClient::~RTSPClient()
{
    stop();
}

RTSPClient::RTSPClient(ImageCallback imageCallback) : callBack_(std::move(imageCallback)) {}
