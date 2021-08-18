#ifndef LMT_PIPELINE_STATE_H
#define LMT_PIPELINE_STATE_H

namespace lmt
{
namespace common
{
enum class PipelineState
{
    started,
    stopped,
    paused,
    starting,
    stopping
};
}
}  // namespace lmt

#endif  // LMT_PIPELINE_STATE_H
