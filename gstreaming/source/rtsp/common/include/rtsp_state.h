#ifndef LMT_RTSP_STATE_H
#define LMT_RTSP_STATE_H

namespace lmt
{
namespace rtsp
{
namespace common
{
enum class RTSPState
{
    started,
    stopped,
    paused,
    starting,
    stopping
};
}
}  // namespace rtsp
}  // namespace lmt

#endif  // LMT_RTSP_STATE_H
