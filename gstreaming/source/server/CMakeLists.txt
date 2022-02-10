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
  PRIVATE ${catkin_LIBRARIES} ${GSTREAMER_VIDEO_LIBRARIES} pthread
  PUBLIC ${OpenCV_LIBS} ${GSTREAMER_LIBRARIES}
         ${GSTREAMER_RTSP_SERVER_LIBRARIES} ${GLIB_LIBRARIES})

target_include_directories(
  gst_server
  PRIVATE include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
          ${GOBJECT_INCLUDE_DIRS} ${GLIB_INCLUDE_DIRS} ${GSTREAMER_INCLUDE_DIRS}
  INTERFACE ${catkin_INCLUDE_DIRS} ${GLIB_INCLUDE_DIRS}
            ${GSTREAMER_INCLUDE_DIRS} include)

target_compile_features(gst_server PUBLIC cxx_std_14)

# RTSP Server
add_library(rtsp_server src/rtsp_server.cpp include/rtsp_server.h
                        src/rtsp_server_util.cpp include/rtsp_server_util.h)

target_link_libraries(rtsp_server PUBLIC gstreaming_common gst_server)

add_dependencies(rtsp_server ${${PROJECT_NAME}_EXPORTED_TARGETS})
