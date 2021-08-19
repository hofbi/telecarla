#ifndef TELECARLA_RPC_ROS_MSG_CALLBACK_H
#define TELECARLA_RPC_ROS_MSG_CALLBACK_H

#include <ros/console.h>
#include <rpc/client.h>
#include <rpc/rpc_error.h>

namespace lmt
{
namespace com
{
template <class ROSMessageType, class RPCMessageType>
class ROSMsgCallback
{
  public:
    ROSMsgCallback(rpc::client& client, std::string functionName) noexcept
        : client_(client), functionName_(std::move(functionName))
    {
    }

    void operator()(const ROSMessageType& rosMsg)
    {
        RPCMessageType rpcMessage(rosMsg);

        try
        {
            client_.send(functionName_, rpcMessage);
        }
        catch (rpc::rpc_error& error)
        {
            ROS_WARN_STREAM("Failed to call function " << error.get_function_name() << " with "
                                                       << error.get_error().as<std::string>());
        }
    }

  private:
    rpc::client& client_;
    std::string functionName_;
};
}  // namespace com
}  // namespace lmt

#endif  // TELECARLA_RPC_ROS_MSG_CALLBACK_H
