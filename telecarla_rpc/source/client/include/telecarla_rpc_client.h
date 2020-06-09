#ifndef TELECARLA_RPC_TELECARLA_RPC_CLIENT_H
#define TELECARLA_RPC_TELECARLA_RPC_CLIENT_H

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <rpc/client.h>

namespace lmt
{
namespace client
{
class TeleCarlaRpcClient
{
  public:
    TeleCarlaRpcClient(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    void update();

  private:
    std::string host_;
    int port_{2002};
    rpc::client client_;
    std::vector<ros::Subscriber> subscribers_;
    ros::Publisher vehicleStatusPublisher_;
};
}  // namespace client
}  // namespace lmt

#endif  // TELECARLA_RPC_TELECARLA_RPC_CLIENT_H
