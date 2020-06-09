#ifndef TELECARLA_GUI_JOYSTICK_PARAMETER_H
#define TELECARLA_GUI_JOYSTICK_PARAMETER_H

namespace lmt::gui
{
struct JoystickParameter
{
    int aWheel;
    int aThrottle;
    int aBrake;
    int bReverse;
    int bMonitoring;
    int bManual;
};
}  // namespace lmt::gui

#endif  // TELECARLA_GUI_JOYSTICK_PARAMETER_H
