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

    ASSERT_FALSE(parameters);
}

TEST(GuiParameter, getVehicleStatusParameters_WhenValidTestFile_CorrectParameters)
{
    GuiParameter unit(vehicleStatusTestFilePath);

    const SDL_Rect expectedParameter{.x = 0, .y = 720, .w = 640, .h = 200};

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

    const SDL_Rect expectedParameter{.x = 0, .y = 720, .w = 640, .h = 200};

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
