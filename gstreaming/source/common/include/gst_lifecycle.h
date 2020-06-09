#ifndef GSTREAMING_GST_LIFECYCLE_H
#define GSTREAMING_GST_LIFECYCLE_H

namespace lmt
{
namespace common
{
class GstLifecycle
{
  public:
    GstLifecycle(int argc, char* argv[]);
    ~GstLifecycle();
};
}  // namespace common
}  // namespace lmt
#endif  // GSTREAMING_GST_LIFECYCLE_H
