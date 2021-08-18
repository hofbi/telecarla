#ifndef TELECARLA_RPC_RPC_MSG_CALLBACK_H
#define TELECARLA_RPC_RPC_MSG_CALLBACK_H

#include <ros/publisher.h>

namespace lmt
{
namespace common
{
template <class RPCMessageType>
class RPCMsgCallback
{
  public:
    explicit RPCMsgCallback(const ros::Publisher& msgPublisher) : msgPublisher_(msgPublisher) {}

    void operator()(const RPCMessageType& rpcMessage) const { msgPublisher_.publish(rpcMessage.toROSMessage()); }

  private:
    ros::Publisher msgPublisher_;
};
}  // namespace common
}  // namespace lmt

#endif  // TELECARLA_RPC_RPC_MSG_CALLBACK_H
