#include "gui_parameter.h"

#include <fstream>

#include <nlohmann/json.hpp>
#include <ros/console.h>

using namespace lmt::gui;

GuiParameter::GuiParameter(const std::string_view& sensorJsonPath)
{
    std::ifstream jsonStream(sensorJsonPath.begin());
    if (!jsonStream.is_open())
    {
        ROS_ERROR_STREAM("Failed to open sensor definition file: " << sensorJsonPath);
        return;
    }

    nlohmann::json matrix = nlohmann::json::parse(jsonStream);
    std::vector<std::vector<SDL_Rect>> tiles;
    int indexY = 0;
    auto heightCompare = [](const SDL_Rect& lhs, const SDL_Rect& rhs) { return lhs.h < rhs.h; };

    for (const auto& row : matrix)
    {
        std::vector<SDL_Rect> tileRow;
        int indexX = 0;
        for (const auto& element : row)
        {
            int width = element.at("w");
            int height = element.at("h");
            tileRow.push_back({indexX, indexY, width, height});
            const SDL_Rect& lastTile = tileRow.back();
            indexX += width;

            if (element.at("type") == "camera")
            {
                cameraParameter_.insert(std::make_pair<std::string, SDL_Rect>(
                    "camera/" + element.at("key").get<std::string>(), SDL_Rect(lastTile)));
            }
            else if (element.at("type") == "vehicle_status")
            {
                vehicleStatusParameter_ = lastTile;
            }
            else if (element.at("type") == "static_text")
            {
                staticTextParameter_ = lastTile;
            }
        }
        windowWidth_ = std::max(windowWidth_, tileRow.back().x + tileRow.back().w);
        indexY = std::max_element(tileRow.begin(), tileRow.end(), heightCompare)->h;
        tiles.push_back(tileRow);
    }
    windowHeight_ = tiles.back().back().y + tiles.back().back().h;
}

const std::map<std::string, SDL_Rect>& GuiParameter::getCameraParameter() const
{
    return cameraParameter_;
}

int GuiParameter::getWindowWidth() const noexcept
{
    return windowWidth_;
}

int GuiParameter::getWindowHeight() const noexcept
{
    return windowHeight_;
}
const std::optional<SDL_Rect>& GuiParameter::getVehicleStatusParameters() const noexcept
{
    return vehicleStatusParameter_;
}

const std::optional<SDL_Rect>& GuiParameter::getStaticTextParameters() const noexcept
{
    return staticTextParameter_;
}
