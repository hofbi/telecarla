#ifndef TELECARLA_GUI_TELECARLA_GUI_H
#define TELECARLA_GUI_TELECARLA_GUI_H

#include <ros/publisher.h>
#include <ros/subscriber.h>

#include "sdl_event_parser.h"
#include "sdl_gui.h"
#include "sdl_lifecycle.h"
#include "sdl_wheel_controller.h"

namespace lmt
{
class TeleCarlaGui
{
  public:
    TeleCarlaGui(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    void update();
    void teleopModeCallback(const telecarla_msgs::TeleopMode& teleopMode);

  private:
    gui::SDL_Lifecycle sdlLifecycle_;
    gui::SDL_GUI sdlGui_;
    std::vector<ros::Subscriber> subscribers_;
    ros::Publisher controlCommandPublisher_;
    ros::Publisher controlOverridePublisher_;
    ros::Publisher enableAutopilotPublisher_;
    gui::SDL_EventParser sdlEventParser_;
    gui::SDL_WheelController sdlWheelController_;
    telecarla_msgs::TeleopMode teleopMode_;
};
}  // namespace lmt

#endif  // TELECARLA_GUI_TELECARLA_GUI_H
