#include "gui_parameter.h"

#include <fstream>

#include <nlohmann/json.hpp>
#include <ros/console.h>

namespace lmt::gui
{
GuiParameter::GuiParameter(const std::string_view& sensorJsonPath)
{
    std::ifstream jsonStream(sensorJsonPath.begin());
    if (!jsonStream.is_open())
    {
        ROS_ERROR_STREAM("Failed to open sensor definition file: " << sensorJsonPath);
        return;
    }

    nlohmann::json matrix = nlohmann::json::parse(jsonStream);
    std::vector<GuiParameter::TilesRow> tiles;
    auto indexY = 0;

    for (const auto& row : matrix)
    {
        GuiParameter::TilesRow tilesRow;
        auto indexX = 0;
        for (const auto& element : row)
        {
            const auto width = static_cast<int>(element.at("w"));
            const auto height = static_cast<int>(element.at("h"));
            tilesRow.push_back({indexX, indexY, width, height});
            const auto& lastTile = tilesRow.back();
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
        windowWidth_ = std::max(windowWidth_, tilesRow.back().x + tilesRow.back().w);
        indexY = getMaxHeight(tilesRow);
        tiles.push_back(tilesRow);
    }
    windowHeight_ = tiles.back().back().y + getMaxHeight(tiles.back());
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

int getMaxHeight(const GuiParameter::TilesRow& tilesRow) noexcept
{
    return std::max_element(tilesRow.cbegin(),
                            tilesRow.cend(),
                            [](const SDL_Rect& lhs, const SDL_Rect& rhs) { return lhs.h < rhs.h; })
        ->h;
}
}  // namespace lmt::gui
