# Basic GStreaming Server
add_library(
  gst_server
  src/gst_server_app_source.cpp
  include/gst_server_app_source.h
  src/gst_server_util.cpp
  include/gst_server_util.h
  include/gst_server_context.h
  src/gst_server_context.cpp
  include/gst_server_encoder.h
  src/gst_server_encoder.cpp)

target_link_libraries(
  gst_server
  PRIVATE ${catkin_LIBRARIES}
  PUBLIC ${OpenCV_LIBS}
  PUBLIC ${GSTREAMER_LIBRARIES}
  PUBLIC ${GSTREAMER_RTSP_SERVER_LIBRARIES}
  PRIVATE ${GSTREAMER_VIDEO_LIBRARIES}
  PUBLIC ${GLIB_LIBRARIES}
  PRIVATE pthread)

target_include_directories(
  gst_server
  PRIVATE include
  PRIVATE ${catkin_INCLUDE_DIRS} SYSTEM
  PRIVATE ${OpenCV_INCLUDE_DIRS} SYSTEM
  PRIVATE ${GOBJECT_INCLUDE_DIRS} SYSTEM
  PRIVATE ${GLIB_INCLUDE_DIRS} SYSTEM
  PRIVATE ${GSTREAMER_INCLUDE_DIRS} SYSTEM
  INTERFACE ${catkin_INCLUDE_DIRS} SYSTEM
  INTERFACE ${GLIB_INCLUDE_DIRS} SYSTEM
  INTERFACE ${GSTREAMER_INCLUDE_DIRS} SYSTEM
  INTERFACE include)

target_compile_features(gst_server PUBLIC cxx_std_14)

# RTSP Server
add_library(rtsp_server src/rtsp_server.cpp include/rtsp_server.h
                        src/rtsp_server_util.cpp include/rtsp_server_util.h)

target_link_libraries(rtsp_server PUBLIC gstreaming_common gst_server)

add_dependencies(rtsp_server ${${PROJECT_NAME}_EXPORTED_TARGETS})
