#include "telecarla_gui.h"

#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "gui_parameter.h"
#include "image_callback.h"
#include "status_callback.h"

using namespace lmt;
using namespace lmt::gui;

TeleCarlaGui::TeleCarlaGui(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : sdlEventParser_([this](const telecarla_msgs::TeleopMode& teleopMode) { teleopModeCallback(teleopMode); }),
      sdlWheelController_(sdlEventParser_)
{
    std::string sensorDefinitionFile;
    if (!pnh.getParam("sensor_definition_file_path", sensorDefinitionFile))
    {
        ROS_ERROR_STREAM("No input topic provided for sensor_definition_file");
        return;
    }

    GuiParameter guiParameter(sensorDefinitionFile);
    sdlGui_ = SDL_GUI(guiParameter);

    for (const auto& topicParam : guiParameter.getCameraParameter())
    {
        std::string inTopic;
        if (!pnh.getParam(std::string(topicParam.first), inTopic))
        {
            ROS_ERROR_STREAM("No input topic provided for camera: " << topicParam.first);
            return;
        }

        subscribers_.push_back(nh.subscribe<sensor_msgs::Image>(
            inTopic, 1, ImageCallback(topicParam.second, [objectPtr = &sdlGui_](auto&&... args) {
                objectPtr->renderImage(std::forward<decltype(args)>(args)...);
            })));
        ROS_INFO_STREAM("Subscribed to image topic for camera " << topicParam.first << ": " << inTopic);
    }

    std::string inTopic;
    if (!pnh.getParam("vehicle_status", inTopic))
    {
        ROS_ERROR_STREAM("No input topic provided for vehicle_status");
        return;
    }

    if (guiParameter.getVehicleStatusParameters())
    {
        subscribers_.push_back(nh.subscribe<carla_msgs::CarlaEgoVehicleStatus>(
            inTopic,
            1,
            StatusCallback(*guiParameter.getVehicleStatusParameters(), [objectPtr = &sdlGui_](auto&&... args) {
                objectPtr->renderStaticText(std::forward<decltype(args)>(args)...);
            })));
        ROS_INFO_STREAM("Subscribed to vehicle status topic: " << inTopic);
    }
    if (sdlWheelController_.isAvailable())
    {
        subscribers_.push_back(nh.subscribe<carla_msgs::CarlaEgoVehicleStatus>(
            inTopic, 1, [&](const carla_msgs::CarlaEgoVehicleStatus::ConstPtr& status) {
                if (teleopMode_.teleopMode != telecarla_msgs::TeleopMode::MANUAL)
                {
                    sdlWheelController_.setTargetWheelPosition(status->control.steer);
                }
            }));
        ROS_INFO_STREAM("Subscribed to vehicle status topic: " << inTopic);
    }

    const auto prefix = "/carla/" + pnh.param("role_name", std::string("ego_vehicle"));
    controlCommandPublisher_ =
        nh.advertise<carla_msgs::CarlaEgoVehicleControl>(prefix + "/vehicle_control_cmd_manual", 1);
    controlOverridePublisher_ = nh.advertise<std_msgs::Bool>(prefix + "/vehicle_control_manual_override", 1);
    enableAutopilotPublisher_ = nh.advertise<std_msgs::Bool>(prefix + "/enable_autopilot", 1);

    sdlGui_.show();
}

void TeleCarlaGui::update()
{
    sdlEventParser_.pollEvents();
    controlCommandPublisher_.publish(sdlEventParser_.getCarlaEgoVehicleControl());
}

void TeleCarlaGui::teleopModeCallback(const telecarla_msgs::TeleopMode& teleopMode)
{
    teleopMode_ = teleopMode;
    switch (teleopMode.teleopMode)
    {
        case telecarla_msgs::TeleopMode::MONITORING: {
            std_msgs::Bool boolMsg;
            boolMsg.data = false;
            controlOverridePublisher_.publish(boolMsg);
            boolMsg.data = true;
            enableAutopilotPublisher_.publish(boolMsg);
            ROS_INFO("Switch to teleop mode: Monitoring");
            break;
        }
        case telecarla_msgs::TeleopMode::MANUAL: {
            std_msgs::Bool boolMsg;
            boolMsg.data = true;
            controlOverridePublisher_.publish(boolMsg);
            boolMsg.data = false;
            enableAutopilotPublisher_.publish(boolMsg);
            ROS_INFO("Switch to teleop mode: Manual");
            break;
        }
    }
}
