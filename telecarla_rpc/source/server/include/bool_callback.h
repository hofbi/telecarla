#ifndef TELECARLA_RPC_SERVER_BOOL_CALLBACK_H
#define TELECARLA_RPC_SERVER_BOOL_CALLBACK_H

#include <ros/publisher.h>

namespace lmt
{
namespace server
{
class BoolCallback
{
  public:
    explicit BoolCallback(const ros::Publisher& msgPublisher);

    void operator()(bool value) const;

  private:
    ros::Publisher msgPublisher_;
};
}  // namespace server
}  // namespace lmt

#endif  // TELECARLA_RPC_SERVER_BOOL_CALLBACK_H
