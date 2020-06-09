#include "sdl_wheel_controller.h"
#include "sdl_event_parser.h"

#include <SDL2/SDL.h>
#include <ros/console.h>
#include <ros/node_handle.h>

namespace lmt::gui
{
SDL_WheelController::SDL_WheelController(const SDL_EventParser& parser) : parser_(parser)
{
    ros::NodeHandle pnh("~");
    if (pnh.param("force_feedback", false) && SDL_NumHaptics() > 0)
    {
        constexpr auto hapticIndex{0};

        wheel_.reset(SDL_HapticOpen(hapticIndex));
        if (wheel_ == nullptr)
        {
            ROS_ERROR_STREAM("Failed to open haptic device " << SDL_GetError());
        }
        else
        {
            ROS_INFO_STREAM("Haptic device available for " << SDL_HapticName(hapticIndex));
        }
    }
}

void SDL_WheelController::setTargetWheelPosition(double targetPosition)
{
    size_t steps{};
    while (steps < 10)
    {
        const auto currentAngle = parser_.getSteerCache();
        const auto controlError = targetPosition - currentAngle;
        SDL_HapticEffect effect;
        if (fabs(controlError) < 0.015)
        {
            break;
        }

        if (controlError < 0)
        {
            effect = makeLeftSteeringEffect();
        }
        else
        {
            effect = makeRightSteeringEffect();
        }

        const auto effectId = SDL_HapticNewEffect(wheel_.get(), &effect);
        SDL_HapticRunEffect(wheel_.get(), effectId, 1);
        SDL_Delay(5);
        SDL_HapticDestroyEffect(wheel_.get(), effectId);
        ++steps;
    }
}

bool SDL_WheelController::isAvailable() const noexcept
{
    return wheel_ != nullptr;
}

SDL_HapticEffect makeLeftSteeringEffect()
{
    return makeSteeringEffect(1);
}

SDL_HapticEffect makeRightSteeringEffect()
{
    return makeSteeringEffect(-1);
}

SDL_HapticEffect makeSteeringEffect(int direction)
{
    SDL_HapticEffect effect;

    // Create the effect
    SDL_memset(&effect, 0, sizeof(SDL_HapticEffect));  // 0 is safe default
    effect.type = SDL_HAPTIC_CONSTANT;
    effect.constant.direction.type = SDL_HAPTIC_CARTESIAN;  // Polar coordinates
    effect.constant.direction.dir[0] = direction;           // Force comes from south
    effect.constant.length = 5000;
    effect.constant.level = 0x4000;
    effect.constant.delay = 0; /**< Delay before starting the effect. */

    /* Envelope */
    effect.constant.attack_length = 1e3; /**< Duration of the attack. */
    effect.constant.attack_level = 1e4;  /**< Level at the start of the attack. */
    effect.constant.fade_length = 0;     /**< Duration of the fade. */
    // effect.constant.fade_level = 1e4;      /**< Level at the end of the fade. */

    return effect;
}
}  // namespace lmt::gui