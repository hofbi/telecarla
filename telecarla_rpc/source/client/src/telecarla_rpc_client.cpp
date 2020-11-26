#include "telecarla_rpc_client.h"

#include "bool_msg_callback.h"
#include "control_command_msg_callback.h"
#include "vehicle_status.h"

using namespace lmt::client;
using namespace lmt::common;

TeleCarlaRpcClient::TeleCarlaRpcClient(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : host_(pnh.param("rpc_host", std::string("localhost"))),
      port_(pnh.param("rpc_port", 2002)),  // NOLINT(readability-magic-numbers)
      client_(host_, port_)
{
    while (client_.get_connection_state() == rpc::client::connection_state::initial)
    {
        ROS_INFO_STREAM("Connecting to " << host_ << ":" << port_ << "...");
    }
    if (client_.get_connection_state() == rpc::client::connection_state::connected)
    {
        ROS_INFO_STREAM("Connected to " << host_ << ":" << port_);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to connect to " << host_ << ":" << port_);
    }

    const auto prefix = "/carla/" + pnh.param("role_name", std::string("ego_vehicle"));

    subscribers_.push_back(nh.subscribe<std_msgs::Bool>(
        prefix + "/enable_autopilot", 1, BoolMsgCallback(client_, "set_enable_autopilot")));
    ROS_INFO_STREAM("Subscribed to topic " << prefix + "/enable_autopilot -> set_enable_autopilot");

    subscribers_.push_back(
        nh.subscribe<std_msgs::Bool>(prefix + "/vehicle_control_manual_override",
                                     1,
                                     BoolMsgCallback(client_, "set_vehicle_control_manual_override")));
    ROS_INFO_STREAM("Subscribed to topic " << prefix +
                                                  "/vehicle_control_manual_override -> "
                                                  "set_vehicle_control_manual_override");

    subscribers_.push_back(nh.subscribe<carla_msgs::CarlaEgoVehicleControl>(prefix + "/vehicle_control_cmd_manual",
                                                                            1,
                                                                            ControlCommandMsgCallback(client_,
                                                                                                      "set_"
                                                                                                      "vehicle_"
                                                                                                      "control_"
                                                                                                      "cmd_"
                                                                                                      "manua"
                                                                                                      "l")));
    ROS_INFO_STREAM("Subscribed to topic " << prefix + "/vehicle_control_cmd_manual -> set_vehicle_control_cmd_manual");

    vehicleStatusPublisher_ = nh.advertise<carla_msgs::CarlaEgoVehicleStatus>(prefix + "/vehicle_status", 1);
}

void TeleCarlaRpcClient::update()
{
    auto status = client_.call("get_vehicle_status").as<VehicleStatus>();

    vehicleStatusPublisher_.publish(status.getMessage());
}
