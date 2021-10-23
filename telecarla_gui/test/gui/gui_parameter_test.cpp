#include "gui_parameter.h"

#include <gtest/gtest.h>

using namespace lmt::gui;

constexpr auto oneCameraTestFilePath = "gui/config/one_camera_config.json";
constexpr auto twoCameraTestFilePath = "gui/config/two_camera_config.json";
constexpr auto threeCameraTestFilePath = "gui/config/three_camera_config.json";
constexpr auto vehicleStatusTestFilePath = "gui/config/vehicle_status_config.json";
constexpr auto staticTextTestFilePath = "gui/config/static_text_config.json";
constexpr auto notExistingCameraTestFilePath = "notExisting.json";

void expectSDLRectEqual(const SDL_Rect& expected, const SDL_Rect& actual) noexcept
{
    EXPECT_EQ(expected.x, actual.x);
    EXPECT_EQ(expected.y, actual.y);
    EXPECT_EQ(expected.w, actual.w);
    EXPECT_EQ(expected.h, actual.h);
}

TEST(GuiParameter, getWindowWidth_WhenOneCameraTestFile)
{
    GuiParameter unit(oneCameraTestFilePath);
    const auto expectedWidth{800};

    const auto actualWidth = unit.getWindowWidth();

    EXPECT_EQ(expectedWidth, actualWidth);
}

TEST(GuiParameter, getWindowWidth_WhenTwoCameraTestFile)
{
    GuiParameter unit(twoCameraTestFilePath);
    const auto expectedWidth{1600};

    const auto actualWidth = unit.getWindowWidth();

    EXPECT_EQ(expectedWidth, actualWidth);
}

TEST(GuiParameter, getWindowWidth_WhenValidThreeCameraTestFile)
{
    GuiParameter unit(threeCameraTestFilePath);
    const auto expectedWidth{1600};

    const auto actualWidth = unit.getWindowWidth();

    EXPECT_EQ(expectedWidth, actualWidth);
}

TEST(GuiParameter, getWindowWidth_WhenNotExistingCameraTestFile)
{
    GuiParameter unit(notExistingCameraTestFilePath);
    const auto expectedWidth{0};

    const auto actualWidth = unit.getWindowWidth();

    EXPECT_EQ(expectedWidth, actualWidth);
}

TEST(GuiParameter, getWindowHeight_WhenOneCameraTestFile)
{
    GuiParameter unit(oneCameraTestFilePath);
    const auto expectedHeight{600};

    const auto actualHeight = unit.getWindowHeight();

    EXPECT_EQ(expectedHeight, actualHeight);
}

TEST(GuiParameter, getWindowHeight_WhenTwoCameraTestFile)
{
    GuiParameter unit(twoCameraTestFilePath);
    const auto expectedHeight{600};

    const auto actualHeight = unit.getWindowHeight();

    EXPECT_EQ(expectedHeight, actualHeight);
}

TEST(GuiParameter, getWindowHeight_WhenValidThreeCameraTestFile)
{
    GuiParameter unit(threeCameraTestFilePath);
    const auto expectedHeight{1200};

    const auto actualHeight = unit.getWindowHeight();

    EXPECT_EQ(expectedHeight, actualHeight);
}

TEST(GuiParameter, getWindowHeight_WhenNotExistingCameraTestFile)
{
    GuiParameter unit(notExistingCameraTestFilePath);
    const auto expectedHeight{0};

    const auto actualHeight = unit.getWindowHeight();

    EXPECT_EQ(expectedHeight, actualHeight);
}

TEST(GuiParameter, getCameraParameterSize_WhenValidOneCameraTestFile)
{
    GuiParameter unit(oneCameraTestFilePath);
    const auto expectedSize{1};

    const auto actualSize = unit.getCameraParameter().size();

    EXPECT_EQ(expectedSize, actualSize);
}

TEST(GuiParameter, getCameraParameterSize_WhenValidThreeCameraTestFile)
{
    GuiParameter unit(threeCameraTestFilePath);
    const auto expectedSize{3};

    const auto actualSize = unit.getCameraParameter().size();

    EXPECT_EQ(expectedSize, actualSize);
}

TEST(GuiParameter, getCameraParameter_WhenValidOneCameraTestFile_Width)
{
    GuiParameter unit(oneCameraTestFilePath);
    const auto expectedWidth{800};

    const auto actualWidth = unit.getCameraParameter().at("camera/front").w;

    EXPECT_EQ(expectedWidth, actualWidth);
}

TEST(GuiParameter, getCameraParameter_WhenValidOneCameraTestFile_Height)
{
    GuiParameter unit(oneCameraTestFilePath);
    const auto expectedHeight{600};

    const auto actualWidth = unit.getCameraParameter().at("camera/front").h;

    EXPECT_EQ(expectedHeight, actualWidth);
}

TEST(GuiParameter, getVehicleStatusParameters_WhenValidTestFileWithoutStatus_Empty)
{
    GuiParameter unit(oneCameraTestFilePath);

    const auto& parameters = unit.getVehicleStatusParameters();

    EXPECT_FALSE(parameters);
}

TEST(GuiParameter, getVehicleStatusParameters_WhenValidTestFile_CorrectParameters)
{
    GuiParameter unit(vehicleStatusTestFilePath);
    const SDL_Rect expectedParameter{.x = 0, .y = 720, .w = 640, .h = 200};

    const auto& actualParameters = unit.getVehicleStatusParameters();

    EXPECT_TRUE(actualParameters);
    expectSDLRectEqual(expectedParameter, *actualParameters);
}

TEST(GuiParameter, getStaticTextParameters_WhenValidTestFile_CorrectParameters)
{
    GuiParameter unit(staticTextTestFilePath);
    const SDL_Rect expectedParameter{.x = 0, .y = 720, .w = 640, .h = 200};

    const auto& actualParameters = unit.getStaticTextParameters();

    EXPECT_TRUE(actualParameters);
    expectSDLRectEqual(expectedParameter, *actualParameters);
}

TEST(GuiParameter, getCameraParameterSize_WhenNotExistingCameraTestFile)
{
    GuiParameter unit(notExistingCameraTestFilePath);

    EXPECT_TRUE(unit.getCameraParameter().empty());
}

TEST(GuiParameter, getMaxHeight_listWithOneElementOfHeight100ShouldHave100)
{
    GuiParameter::TilesRow tilesRow{{.x = 0, .y = 0, .w = 200, .h = 100}};  // NOLINT(readability-magic-numbers)

    const auto maxHeight = getMaxHeight(tilesRow);

    EXPECT_EQ(100, maxHeight);
}

TEST(GuiParameter, getMaxHeight_listWithMaxHeight300ShouldHave300)
{
    GuiParameter::TilesRow tilesRow{{.x = 0, .y = 0, .w = 200, .h = 100},   // NOLINT(readability-magic-numbers)
                                    {.x = 0, .y = 0, .w = 200, .h = 300},   // NOLINT(readability-magic-numbers)
                                    {.x = 0, .y = 0, .w = 200, .h = 200}};  // NOLINT(readability-magic-numbers)

    const auto maxHeight = getMaxHeight(tilesRow);

    EXPECT_EQ(300, maxHeight);
}
