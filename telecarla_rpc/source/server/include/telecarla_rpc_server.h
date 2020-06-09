#ifndef TELECARLA_RPC_TELECARLA_RPC_SERVER_H
#define TELECARLA_RPC_TELECARLA_RPC_SERVER_H

#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <rpc/server.h>

namespace lmt
{
namespace server
{
class TeleCarlaRpcServer
{
  public:
    TeleCarlaRpcServer(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    void run();
    void stop();

  private:
    void vehicleStatusCallback(const carla_msgs::CarlaEgoVehicleStatusConstPtr& vehicleStatus);

  private:
    rpc::server server_;
    ros::Subscriber vehicleStatusSubscriber_;
    carla_msgs::CarlaEgoVehicleStatusConstPtr egoVehicleStatus_;
};
}  // namespace server
}  // namespace lmt

#endif  // TELECARLA_RPC_TELECARLA_RPC_SERVER_H
