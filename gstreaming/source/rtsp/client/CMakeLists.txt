add_library(rtsp_client src/rtsp_client.cpp include/rtsp_client.h)

target_link_libraries(
  rtsp_client
  PUBLIC rtsp_common
  PRIVATE gstreaming_common
  PUBLIC ${GSTREAMER_LIBRARIES}
  PUBLIC ${GLIB_LIBRARIES}
  PUBLIC ${GSTREAMER_APP_LIBRARIES}
  PRIVATE pthread)

target_include_directories(
  rtsp_client
  PRIVATE include
  PRIVATE ${GOBJECT_INCLUDE_DIRS}
  PRIVATE ${GLIB_INCLUDE_DIRS}
  PRIVATE ${GSTREAMER_INCLUDE_DIRS} SYSTEM
  INTERFACE ${GLIB_INCLUDE_DIRS} SYSTEM
  INTERFACE ${GSTREAMER_INCLUDE_DIRS} SYSTEM
  INTERFACE include)
