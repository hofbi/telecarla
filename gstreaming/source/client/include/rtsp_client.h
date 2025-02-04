#ifndef _LMT_RTSP_CLIENT_HH_
#define _LMT_RTSP_CLIENT_HH_

#include <functional>
#include <string>

#include <gst/app/gstappsink.h>
#include <gst/gstpipeline.h>

#include "pipeline_state.h"

namespace lmt
{
namespace client
{
class RTSPClient
{
  public:
    using ImageCallback = std::function<void(char* data, int size, int width, int height)>;

    explicit RTSPClient(ImageCallback imageCallback);
    ~RTSPClient();

    RTSPClient(const RTSPClient& rhs) = delete;
    RTSPClient& operator=(const RTSPClient& rhs) = delete;

    RTSPClient(RTSPClient&& rhs) = delete;
    RTSPClient& operator=(RTSPClient&& rhs) = delete;

    common::PipelineState start(const std::string& serverHost, int serverPort, const std::string& serverMount);
    void stop();
    void resume();

  private:
    static GstFlowReturn onNewSampleFromSink(GstAppSink* appSink, RTSPClient* data);

  private:
    ImageCallback callBack_;
    GstElement* pipeline_{nullptr};
    GstAppSink* receiverAppSink_{nullptr};
    common::PipelineState state_{common::PipelineState::stopped};
};
}  // namespace client
}  // namespace lmt

#endif /* _LMT_RTSP_CLIENT_HH_ */
