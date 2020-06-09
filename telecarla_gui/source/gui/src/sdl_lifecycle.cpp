#include "sdl_lifecycle.h"

#include <SDL2/SDL.h>
#include <ros/console.h>

using namespace lmt::gui;

SDL_Lifecycle::SDL_Lifecycle()
{
    if (SDL_Init(SDL_INIT_EVERYTHING) != 0)
    {
        ROS_ERROR_STREAM("SDL_Init Error: " << SDL_GetError());
        return;
    }
}

SDL_Lifecycle::~SDL_Lifecycle()
{
    SDL_Quit();
}
