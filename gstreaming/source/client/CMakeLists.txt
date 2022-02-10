add_library(rtsp_client src/rtsp_client.cpp include/rtsp_client.h)

target_link_libraries(
  rtsp_client
  PUBLIC gstreaming_common ${GSTREAMER_LIBRARIES} ${GLIB_LIBRARIES}
         ${GSTREAMER_APP_LIBRARIES}
  PRIVATE pthread)

target_include_directories(
  rtsp_client
  PRIVATE include ${GOBJECT_INCLUDE_DIRS} ${GLIB_INCLUDE_DIRS}
          ${GSTREAMER_INCLUDE_DIRS}
  INTERFACE ${GLIB_INCLUDE_DIRS} ${GSTREAMER_INCLUDE_DIRS} include)
