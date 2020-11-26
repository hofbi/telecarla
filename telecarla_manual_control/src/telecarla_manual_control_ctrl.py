#!/usr/bin/env python
"""
Welcome to CARLA ROS manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    AD           : steer
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    B            : toggle manual control

    ESC          : quit
"""

from __future__ import print_function

import sys

import math
import rospy
from carla_msgs.msg import CarlaEgoVehicleControl
from std_msgs.msg import Bool

if sys.version_info >= (3, 0):
    from configparser import ConfigParser
else:
    from ConfigParser import RawConfigParser as ConfigParser

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_LEFT
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SPACE
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_s
    from pygame.locals import K_w
    from pygame.locals import K_b
except ImportError:
    raise RuntimeError("cannot import pygame, make sure pygame package is installed")


class BaseControl(object):
    """
    Handle input events
    """

    def __init__(self, role_name):
        self.role_name = role_name

        self.vehicle_control_manual_override_publisher = rospy.Publisher(
            "/carla/{}/vehicle_control_manual_override".format(self.role_name),
            Bool,
            queue_size=1,
            latch=True,
        )
        self.vehicle_control_manual_override = True
        self.auto_pilot_enable_publisher = rospy.Publisher(
            "/carla/{}/enable_autopilot".format(self.role_name), Bool, queue_size=1
        )
        self.vehicle_control_publisher = rospy.Publisher(
            "/carla/{}/vehicle_control_cmd_manual".format(self.role_name),
            CarlaEgoVehicleControl,
            queue_size=1,
        )
        self._autopilot_enabled = False
        self._control = CarlaEgoVehicleControl()
        self.set_autopilot(self._autopilot_enabled)
        self._steer_cache = 0.0
        self.set_vehicle_control_manual_override(
            self.vehicle_control_manual_override
        )  # disable manual override

    def __del__(self):
        self.auto_pilot_enable_publisher.unregister()
        self.vehicle_control_publisher.unregister()
        self.vehicle_control_manual_override_publisher.unregister()

    def set_vehicle_control_manual_override(self, enable):
        """
        Set the manual control override
        """
        self.vehicle_control_manual_override_publisher.publish((Bool(data=enable)))

    def set_autopilot(self, enable):
        """
        enable/disable the autopilot
        """
        self.auto_pilot_enable_publisher.publish(Bool(data=enable))

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


class KeyboardControl(BaseControl):
    """
    Handle input events
    """

    def parse_events(self, clock):
        """
        parse an input event
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_b:
                    self.vehicle_control_manual_override = (
                        not self.vehicle_control_manual_override
                    )
                    self.set_vehicle_control_manual_override(
                        self.vehicle_control_manual_override
                    )
                if event.key == K_q:
                    self._control.gear = 1 if self._control.reverse else -1
                elif event.key == K_p:
                    self._autopilot_enabled = not self._autopilot_enabled
                    self.set_autopilot(self._autopilot_enabled)
        if not self._autopilot_enabled and self.vehicle_control_manual_override:
            self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
            self._control.reverse = self._control.gear < 0
            self.vehicle_control_publisher.publish(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds):
        """
        parse key events
        """
        self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = keys[K_SPACE]


class PWSteering(BaseControl):
    """
    Handle pedals and wheel input events
    """

    def __init__(self, role_name, config_file):
        super(PWSteering, self).__init__(role_name)

        # initialize steering wheel
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        if joystick_count > 1:
            raise ValueError("Please Connect Just One Joystick")

        self._joystick = pygame.joystick.Joystick(0)
        self._joystick.init()

        self._parser = ConfigParser()
        self._parser.read(config_file)
        self._steer_idx = int(self._parser.get("G29Pedal", "steering_wheel"))
        self._throttle_idx = int(self._parser.get("G29Pedal", "throttle"))
        self._brake_idx = int(self._parser.get("G29Pedal", "brake"))

        self._k1 = float(self._parser.get("G29Pedal", "k1"))
        self._k2 = float(self._parser.get("G29Pedal", "k2"))
        self._sensitivity = float(self._parser.get("G29Pedal", "sensitivity"))

    def parse_events(self, clock):
        """
        parse an input event
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == int(self._parser.get("G29WheelButton", "handbrake")):
                    self._control.hand_brake = not self._control.hand_brake
                elif event.button == int(self._parser.get("G29WheelButton", "reverse")):
                    self._control.gear = 1 if self._control.reverse else -1
        if not self._autopilot_enabled:
            self._parse_vehicle_wheel()
            self._control.reverse = self._control.gear < 0
            self.vehicle_control_publisher.publish(self._control)

    def _parse_vehicle_wheel(self):
        num_axes = self._joystick.get_numaxes()
        js_inputs = [float(self._joystick.get_axis(i)) for i in range(num_axes)]

        # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
        # For the steering, it seems fine as it is
        self._control.steer = self._k1 * math.tan(
            self._sensitivity * js_inputs[self._steer_idx]
        )
        # FIXME: brake and throttle get mixed up here. The issue can be traced back to the wheel config file
        # this problem seems to be present in all implementations
        # fixing this issue would result in compatibility issues with other config files
        self._control.brake = self._calculate_pedal_value(js_inputs[self._throttle_idx])
        self._control.throttle = self._calculate_pedal_value(js_inputs[self._brake_idx])

    def _calculate_pedal_value(self, js_input):
        if abs(js_input) < 0.00001:
            # an exact value of zero only happens during startup
            # we asssume, the pedal is not pressed and return zero
            # to avoid stuck pedals
            return 0
        return self._min_max(
            self._k2 + (2.05 * math.log10(-0.7 * js_input + 1.4) - 1.2) / 0.92
        )

    @staticmethod
    def _min_max(value, min_val=0, max_val=1):
        if value <= min_val:
            value = min_val
        elif value > max_val:
            value = max_val
        return value


def main():
    """
    main function
    """
    rospy.init_node("telecarla_manual_control_ctrl", anonymous=True)

    role_name = rospy.get_param("~role_name", "ego_vehicle")
    keyboard_control = bool(rospy.get_param("~keyboard_control", True))
    if not rospy.has_param("~config_file"):
        rospy.logerr("No config file provided")

    config_file = rospy.get_param("~config_file")
    resolution = {"width": 1, "height": 1}

    pygame.init()
    pygame.font.init()
    pygame.display.set_caption("CARLA ROS teleop manual control controller")
    world = None
    try:
        pygame.display.set_mode(
            (resolution["width"], resolution["height"]),
            pygame.HWSURFACE | pygame.DOUBLEBUF,
        )

        if keyboard_control:
            controller = KeyboardControl(role_name)
            rospy.loginfo("Keyboard Control")
        else:
            controller = PWSteering(role_name, config_file)
            rospy.loginfo("PW Steering Control")

        clock = pygame.time.Clock()

        while not rospy.core.is_shutdown():
            clock.tick_busy_loop(20)
            if controller.parse_events(clock):
                return
            pygame.display.flip()

    finally:
        if world is not None:
            world.destroy()
        pygame.quit()


if __name__ == "__main__":
    main()
