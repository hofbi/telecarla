#ifndef TELECARLA_RPC_BOOL_MSG_CALLBACK_H
#define TELECARLA_RPC_BOOL_MSG_CALLBACK_H

#include <std_msgs/Bool.h>

namespace rpc
{
class client;
}
namespace lmt
{
namespace client
{
class BoolMsgCallback
{
  public:
    BoolMsgCallback(rpc::client& client, std::string functionName) noexcept;

    void operator()(const std_msgs::BoolConstPtr& boolConstPtr);

  private:
    rpc::client& client_;
    std::string functionName_;
};
}  // namespace client
}  // namespace lmt

#endif  // TELECARLA_RPC_BOOL_MSG_CALLBACK_H
