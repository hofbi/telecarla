#include "sdl_gui.h"

#include <SDL2/SDL.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/console.h>

#include "gui_parameter.h"

using namespace lmt::gui;

SDL_GUI::SDL_GUI(const GuiParameter& guiParameter) : staticTextParameters_(guiParameter.getStaticTextParameters())
{
    window_.reset(SDL_CreateWindow("Teleop GUI",
                                   SDL_WINDOWPOS_CENTERED,
                                   SDL_WINDOWPOS_CENTERED,
                                   guiParameter.getWindowWidth(),
                                   guiParameter.getWindowHeight(),
                                   SDL_WINDOW_SHOWN | SDL_WINDOW_OPENGL));
    if (window_ == nullptr)
    {
        ROS_ERROR_STREAM("SDL_CreateWindow Error: " << SDL_GetError());
        return;
    }

    // -1 means it automatically searches the gpu
    renderer_.reset(SDL_CreateRenderer(window_.get(), -1, SDL_RENDERER_ACCELERATED));
    if (renderer_ == nullptr)
    {
        ROS_ERROR_STREAM("SDL_CreateRenderer Error: " << SDL_GetError());
        return;
    }

    texture_.reset(SDL_CreateTexture(renderer_.get(),
                                     SDL_PIXELFORMAT_RGB24,
                                     SDL_TEXTUREACCESS_STREAMING,
                                     guiParameter.getWindowWidth(),
                                     guiParameter.getWindowHeight()));
    if (texture_ == nullptr)
    {
        ROS_ERROR_STREAM("SDL_CreateTexture Error: " << SDL_GetError());
        return;
    }
}

void SDL_GUI::show()
{
    static const TextLines staticText{{"Control Commands:", ""},
                                      {},
                                      {"Throttle:", "W || Up    || Right Pedal"},
                                      {"  Brake:", "S || Down  || Middle Pedal"},
                                      {"  Left:", "A || Left  || Steer Left"},
                                      {"  Right:", "D || Right || Steer Right"},
                                      {},
                                      {"  Reverse:", "Q || Right Flap"},
                                      {},
                                      {"Teleop Modes:", ""},
                                      {},
                                      {"  Monitoring:", "0 || Circle"},
                                      {"  Manual:", "1 || X"}};

    SDL_RenderClear(renderer_.get());  // replace everything with the drawing color (default: #000000);
    if (staticTextParameters_)
    {
        renderStaticText(*staticTextParameters_, staticText);
    }
    SDL_RenderPresent(renderer_.get());  // show window*/
}

void SDL_GUI::renderImage(const SDL_Rect& pos, const sensor_msgs::ImageConstPtr& msg, int imageFrequency) const
{
    cv::Mat image;
    cv::resize(cv_bridge::toCvShare(msg, "rgb8")->image, image, cv::Size(pos.w, pos.h));

    std::ostringstream text;
    text << "Cam: " << msg->header.frame_id.substr(msg->header.frame_id.find_last_of('/') + 1)
         << " | Size: " << msg->width << "x" << msg->height << " | FPS: " << imageFrequency;
    const cv::Point bottomLeftOfText{15, 30};  // NOLINT(readability-magic-numbers)
    const auto fontScale{1.5};
    const cv::Scalar grey{125, 125, 125};
    cv::putText(image, text.str(), bottomLeftOfText, cv::FONT_HERSHEY_PLAIN, fontScale, grey, 2);

    renderCvMat(pos, image);
}

void SDL_GUI::renderStaticText(const SDL_Rect& pos, const TextLines& textLines) const
{
    cv::Mat image(pos.h, pos.w, CV_8UC3, cv::Scalar(0, 0, 0));

    cv::Point bottomLeftOfText{15, 30};  // NOLINT(readability-magic-numbers)
    const cv::Scalar grey{125, 125, 125};
    const auto verticalOffsetBetweenTextLines{20};

    for (const auto& line : textLines)
    {
        cv::putText(image, line.first.data(), bottomLeftOfText, cv::FONT_HERSHEY_PLAIN, 1.0, grey, 1);

        cv::Point horizontalOffset{150, 0};  // NOLINT(readability-magic-numbers)
        cv::putText(
            image, line.second.data(), bottomLeftOfText + horizontalOffset, cv::FONT_HERSHEY_PLAIN, 1.0, grey, 1);
        bottomLeftOfText.y += verticalOffsetBetweenTextLines;
    }

    renderCvMat(pos, image);
}

void SDL_GUI::renderCvMat(const SDL_Rect& pos, const cv::Mat& image) const
{
    SDL_UpdateTexture(texture_.get(), &pos, image.data, image.step[0]);

    SDL_RenderCopy(renderer_.get(), texture_.get(), &pos, &pos);
    SDL_RenderPresent(renderer_.get());
}
