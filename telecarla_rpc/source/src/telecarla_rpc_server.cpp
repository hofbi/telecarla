#include "telecarla_rpc_server.h"

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "bool.h"
#include "rpc_msg_callback.h"
#include "vehicle_status.h"

using namespace lmt;

TeleCarlaRpcServer::TeleCarlaRpcServer(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : server_(pnh.param("rpc_port", 2002)  // NOLINT(readability-magic-numbers)
      )
{
    ROS_INFO_STREAM("Opening RPC Server at port " << pnh.param("rpc_port", 2002));  // NOLINT(readability-magic-numbers)

    const auto prefix = "/carla/" + pnh.param("role_name", std::string("ego_vehicle"));

    server_.bind("set_enable_autopilot",
                 common::RPCMsgCallback<data::Bool>(nh.advertise<std_msgs::Bool>(prefix + "/enable_autopilot", 1)));
    ROS_INFO_STREAM("Provide RPC call set_enable_autopilot -> " << prefix << "/enable_autopilot");
    server_.bind("set_vehicle_control_manual_override",
                 common::RPCMsgCallback<data::Bool>(
                     nh.advertise<std_msgs::Bool>(prefix + "/vehicle_control_manual_override", 1)));
    ROS_INFO_STREAM("Provide RPC call set_vehicle_control_manual_override -> " << prefix
                                                                               << "/vehicle_control_manual_override");
    server_.bind("set_vehicle_control_cmd_manual",
                 common::RPCMsgCallback<data::ControlCommands>(
                     nh.advertise<carla_msgs::CarlaEgoVehicleControl>(prefix + "/vehicle_control_cmd_manual", 1)));
    ROS_INFO_STREAM("Provide RPC call set_vehicle_control_cmd_manual -> " << prefix << "/vehicle_control_cmd_manual");

    server_.bind("get_vehicle_status", [this]() { return data::VehicleStatus(egoVehicleStatus_); });
    ROS_INFO("Provide RPC call get_vehicle_status");
    vehicleStatusSubscriber_ = nh.subscribe<carla_msgs::CarlaEgoVehicleStatus>(
        prefix + "/vehicle_status", 1, &TeleCarlaRpcServer::vehicleStatusCallback, this);
    ROS_INFO_STREAM("Subscribed to " << prefix << "/vehicle_status");
}

void TeleCarlaRpcServer::run()
{
    server_.async_run();
}

void TeleCarlaRpcServer::stop()
{
    server_.close_sessions();
    server_.stop();
}

void TeleCarlaRpcServer::vehicleStatusCallback(const carla_msgs::CarlaEgoVehicleStatusConstPtr& vehicleStatus)
{
    egoVehicleStatus_ = vehicleStatus;
}
