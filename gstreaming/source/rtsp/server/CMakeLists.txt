add_library(
  rtsp_server
  src/rtsp_server.cpp
  include/rtsp_server.h
  src/rtsp_server_app_source.cpp
  include/rtsp_server_app_source.h
  src/gst_rtsp_server_util.cpp
  include/gst_rtsp_server_util.h
  include/rtsp_server_context.h
  src/rtsp_server_context.cpp
  include/rtsp_server_encoder.h
  src/rtsp_server_encoder.cpp)

target_link_libraries(
  rtsp_server
  PUBLIC rtsp_common
  PRIVATE gstreaming_common
  PRIVATE ${catkin_LIBRARIES}
  PUBLIC ${OpenCV_LIBS}
  PUBLIC ${GSTREAMER_LIBRARIES}
  PUBLIC ${GSTREAMER_RTSP_SERVER_LIBRARIES}
  PRIVATE ${GSTREAMER_VIDEO_LIBRARIES}
  PUBLIC ${GLIB_LIBRARIES}
  PRIVATE pthread)

target_include_directories(
  rtsp_server
  PRIVATE include
  PRIVATE ${catkin_INCLUDE_DIRS}
  PRIVATE ${OpenCV_INCLUDE_DIRS}
  PRIVATE ${GOBJECT_INCLUDE_DIRS}
  PRIVATE ${GLIB_INCLUDE_DIRS}
  PRIVATE ${GSTREAMER_INCLUDE_DIRS} SYSTEM
  INTERFACE ${GLIB_INCLUDE_DIRS} SYSTEM
  INTERFACE ${GSTREAMER_INCLUDE_DIRS} SYSTEM
  INTERFACE include)

target_compile_features(rtsp_server PUBLIC cxx_std_14)

add_dependencies(rtsp_server ${${PROJECT_NAME}_EXPORTED_TARGETS})
