#include "gui_parameter.h"

#include <gtest/gtest.h>

using namespace lmt::gui;

constexpr auto oneCameraTestFilePath = "gui/config/one_camera_config.json";
constexpr auto twoCameraTestFilePath = "gui/config/two_camera_config.json";
constexpr auto threeCameraTestFilePath = "gui/config/three_camera_config.json";
constexpr auto vehicleStatusTestFilePath = "gui/config/vehicle_status_config.json";
constexpr auto staticTextTestFilePath = "gui/config/static_text_config.json";
constexpr auto notExistingCameraTestFilePath = "notExisting.json";

TEST(GuiParameter, getWindowWidth_WhenOneCameraTestFile)
{
    GuiParameter unit(oneCameraTestFilePath);
    auto expectedWidth = 800;

    auto actualWidth = unit.getWindowWidth();

    EXPECT_EQ(expectedWidth, actualWidth);
}

TEST(GuiParameter, getWindowWidth_WhenTwoCameraTestFile)
{
    GuiParameter unit(twoCameraTestFilePath);
    auto expectedWidth = 1600;

    auto actualWidth = unit.getWindowWidth();

    EXPECT_EQ(expectedWidth, actualWidth);
}

TEST(GuiParameter, getWindowWidth_WhenValidThreeCameraTestFile)
{
    GuiParameter unit(threeCameraTestFilePath);
    auto expectedWidth = 1600;

    auto actualWidth = unit.getWindowWidth();

    EXPECT_EQ(expectedWidth, actualWidth);
}

TEST(GuiParameter, getWindowWidth_WhenNotExistingCameraTestFile)
{
    GuiParameter unit(notExistingCameraTestFilePath);
    auto expectedWidth = 0;

    auto actualWidth = unit.getWindowWidth();

    EXPECT_EQ(expectedWidth, actualWidth);
}

TEST(GuiParameter, getWindowHeight_WhenOneCameraTestFile)
{
    GuiParameter unit(oneCameraTestFilePath);
    auto expectedHeight = 600;

    auto actualHeight = unit.getWindowHeight();

    EXPECT_EQ(expectedHeight, actualHeight);
}

TEST(GuiParameter, getWindowHeight_WhenTwoCameraTestFile)
{
    GuiParameter unit(twoCameraTestFilePath);
    auto expectedHeight = 600;

    auto actualHeight = unit.getWindowHeight();

    EXPECT_EQ(expectedHeight, actualHeight);
}

TEST(GuiParameter, getWindowHeight_WhenValidThreeCameraTestFile)
{
    GuiParameter unit(threeCameraTestFilePath);
    auto expectedHeight = 1200;

    auto actualHeight = unit.getWindowHeight();

    EXPECT_EQ(expectedHeight, actualHeight);
}

TEST(GuiParameter, getWindowHeight_WhenNotExistingCameraTestFile)
{
    GuiParameter unit(notExistingCameraTestFilePath);
    auto expectedHeight = 0;

    auto actualHeight = unit.getWindowHeight();

    EXPECT_EQ(expectedHeight, actualHeight);
}

TEST(GuiParameter, getCameraParameterSize_WhenValidOneCameraTestFile)
{
    GuiParameter unit(oneCameraTestFilePath);
    auto expectedSize = 1;

    auto actualSize = unit.getCameraParameter().size();

    EXPECT_EQ(expectedSize, actualSize);
}

TEST(GuiParameter, getCameraParameterSize_WhenValidThreeCameraTestFile)
{
    GuiParameter unit(threeCameraTestFilePath);
    auto expectedSize = 3;

    auto actualSize = unit.getCameraParameter().size();

    EXPECT_EQ(expectedSize, actualSize);
}

TEST(GuiParameter, getCameraParameter_WhenValidOneCameraTestFile_Width)
{
    GuiParameter unit(oneCameraTestFilePath);
    auto expectedWidth = 800;

    auto actualWidth = unit.getCameraParameter().at("camera/front").w;

    EXPECT_EQ(expectedWidth, actualWidth);
}

TEST(GuiParameter, getCameraParameter_WhenValidOneCameraTestFile_Height)
{
    GuiParameter unit(oneCameraTestFilePath);
    auto expectedHeight = 600;

    auto actualWidth = unit.getCameraParameter().at("camera/front").h;

    EXPECT_EQ(expectedHeight, actualWidth);
}

TEST(GuiParameter, getVehicleStatusParameters_WhenValidTestFileWithoutStatus_Empty)
{
    GuiParameter unit(oneCameraTestFilePath);

    const auto& parameters = unit.getVehicleStatusParameters();

    ASSERT_FALSE(parameters);
}

TEST(GuiParameter, getVehicleStatusParameters_WhenValidTestFile_CorrectParameters)
{
    GuiParameter unit(vehicleStatusTestFilePath);
    SDL_Rect expectedParameter{0, 720, 640, 200};

    const auto& actualParameters = unit.getVehicleStatusParameters();

    ASSERT_TRUE(actualParameters);
    EXPECT_EQ(expectedParameter.x, actualParameters->x);
    EXPECT_EQ(expectedParameter.y, actualParameters->y);
    EXPECT_EQ(expectedParameter.w, actualParameters->w);
    EXPECT_EQ(expectedParameter.h, actualParameters->h);
}

TEST(GuiParameter, getStaticTextParameters_WhenValidTestFile_CorrectParameters)
{
    GuiParameter unit(staticTextTestFilePath);
    SDL_Rect expectedParameter{0, 720, 640, 200};

    const auto& actualParameters = unit.getStaticTextParameters();

    ASSERT_TRUE(actualParameters);
    EXPECT_EQ(expectedParameter.x, actualParameters->x);
    EXPECT_EQ(expectedParameter.y, actualParameters->y);
    EXPECT_EQ(expectedParameter.w, actualParameters->w);
    EXPECT_EQ(expectedParameter.h, actualParameters->h);
}

TEST(GuiParameter, getCameraParameterSize_WhenNotExistingCameraTestFile)
{
    GuiParameter unit(notExistingCameraTestFilePath);

    EXPECT_TRUE(unit.getCameraParameter().empty());
}
