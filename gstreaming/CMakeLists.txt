cmake_minimum_required(VERSION 3.0)
project(gstreaming)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Package options
set(CMAKE_CXX_FLAGS_RELEASE
    "-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO")
set(CMAKE_CXX_FLAGS_DEBUG
    "${CMAKE_CXX_FLAGS_DEBUG} -DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_DEBUG"
)

# Catkin
find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             std_msgs
             sensor_msgs
             cv_bridge
             image_transport
             roslaunch
             dynamic_reconfigure)

generate_dynamic_reconfigure_options(cfg/RateControl.cfg)

# Third party
find_package(OpenCV REQUIRED)

# GStreamer
find_package(
  GStreamer
  COMPONENTS GSTREAMER_NET GSTREAMER_RTSP_SERVER
  REQUIRED)
find_package(
  GLIB
  COMPONENTS gobject
  REQUIRED)

# Package
catkin_package(DEPENDS OpenCV GSTREAMER GLIB dynamic_reconfigure)

# Source
add_subdirectory(source)

# Test
if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
