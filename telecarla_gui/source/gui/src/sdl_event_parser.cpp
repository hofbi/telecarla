#include "sdl_event_parser.h"

#include <cmath>

#include <SDL2/SDL.h>
#include <ros/ros.h>

using namespace lmt::gui;

const carla_msgs::CarlaEgoVehicleControl& SDL_EventParser::getCarlaEgoVehicleControl() const noexcept
{
    return carlaEgoVehicleControl_;
}

void SDL_EventParser::pollEvents()
{
    SDL_Event event;

    while (SDL_PollEvent(&event) == 1)  // No user event detected
    {
        if (event.type == SDL_QUIT || (event.type == SDL_KEYUP && event.key.keysym.sym == SDLK_ESCAPE))
        {
            ros::shutdown();
        }

        if (event.type == SDL_KEYDOWN)
        {
            if (event.key.keysym.sym == SDLK_w || event.key.keysym.sym == SDLK_UP)
            {
                carlaEgoVehicleControl_.throttle = 1.0;
            }
            else if (event.key.keysym.sym == SDLK_s || event.key.keysym.sym == SDLK_DOWN)
            {
                carlaEgoVehicleControl_.brake = 1.0;
            }
            else if (event.key.keysym.sym == SDLK_a || event.key.keysym.sym == SDLK_LEFT)
            {
                steerCache_.left();
            }
            else if (event.key.keysym.sym == SDLK_d || event.key.keysym.sym == SDLK_RIGHT)
            {
                steerCache_.right();
            }
        }

        if (event.type == SDL_KEYUP)
        {
            if (event.key.keysym.sym == SDLK_w || event.key.keysym.sym == SDLK_UP)
            {
                carlaEgoVehicleControl_.throttle = 0.0;
            }
            else if (event.key.keysym.sym == SDLK_s || event.key.keysym.sym == SDLK_DOWN)
            {
                carlaEgoVehicleControl_.brake = 0.0;
            }
            else if (event.key.keysym.sym == SDLK_a || event.key.keysym.sym == SDLK_LEFT ||
                     event.key.keysym.sym == SDLK_d || event.key.keysym.sym == SDLK_RIGHT)
            {
                steerCache_.reset();
            }
            else if (event.key.keysym.sym == SDLK_q)
            {
                carlaEgoVehicleControl_.gear = carlaEgoVehicleControl_.reverse ? 1 : -1;
            }
            else if (event.key.keysym.sym == SDLK_0)
            {
                switchTeleopMode(telecarla_msgs::TeleopMode::MONITORING);
            }
            else if (event.key.keysym.sym == SDLK_1)
            {
                switchTeleopMode(telecarla_msgs::TeleopMode::MANUAL);
            }
        }

        if (joystick_ != nullptr)
        {
            if (event.type == SDL_JOYBUTTONUP)
            {
                if (event.jbutton.button == joystickParameter_.bReverse)  // Right Flap
                {
                    carlaEgoVehicleControl_.gear = carlaEgoVehicleControl_.reverse ? 1 : -1;
                }
                if (event.jbutton.button == joystickParameter_.bMonitoring)  // O
                {
                    switchTeleopMode(telecarla_msgs::TeleopMode::MONITORING);
                }
                if (event.jbutton.button == joystickParameter_.bManual)  // X
                {
                    switchTeleopMode(telecarla_msgs::TeleopMode::MANUAL);
                }
            }
            if (event.type == SDL_JOYAXISMOTION)
            {
                if (event.jaxis.axis == joystickParameter_.aWheel)  // Wheel
                {
                    steerCache_.set(wheelFunction_(event.jaxis.value));
                }
                if (event.jaxis.axis == joystickParameter_.aThrottle)  // Throttle
                {
                    carlaEgoVehicleControl_.throttle = pedalFunction_(event.jaxis.value);
                }
                if (event.jaxis.axis == joystickParameter_.aBrake)  // Brake
                {
                    carlaEgoVehicleControl_.brake = pedalFunction_(event.jaxis.value);
                }
            }
        }
    }
    carlaEgoVehicleControl_.steer = steerCache_.get();
    carlaEgoVehicleControl_.reverse = carlaEgoVehicleControl_.gear < 0;
}

SDL_EventParser::SDL_EventParser(TeleopModeCallback teleopModeCallback)
    : teleopModeCallback_(std::move(teleopModeCallback))
{
    ROS_INFO("Keyboard control available");

    if (SDL_NumJoysticks() > 0)
    {
        SDL_JoystickEventState(SDL_ENABLE);
        joystick_.reset(SDL_JoystickOpen(0));
        if (joystick_ == nullptr)
        {
            ROS_ERROR_STREAM("Failed to open joystick " << SDL_GetError());
        }
        else
        {
            ROS_INFO_STREAM("Joystick control available for " << SDL_JoystickName(joystick_.get()));
        }
    }

    ros::NodeHandle pnh("~");
    pnh.getParam("joy_axis/steering_wheel", joystickParameter_.aWheel);
    pnh.getParam("joy_axis/throttle", joystickParameter_.aThrottle);
    pnh.getParam("joy_axis/brake", joystickParameter_.aBrake);
    pnh.getParam("joy_button/reverse", joystickParameter_.bReverse);
    pnh.getParam("joy_button/monitoring", joystickParameter_.bMonitoring);
    pnh.getParam("joy_button/manual", joystickParameter_.bManual);
}

void SDL_EventParser::switchTeleopMode(uint8_t mode)
{
    telecarla_msgs::TeleopMode teleopMode;
    teleopMode.teleopMode = mode;
    teleopModeCallback_(teleopMode);
}

double SDL_EventParser::getSteerCache() const noexcept
{
    return steerCache_.get();
}
