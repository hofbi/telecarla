add_library(gstreaming_common include/gst_lifecycle.h src/gst_lifecycle.cpp)

target_link_libraries(
  gstreaming_common
  PRIVATE ${GSTREAMER_LIBRARIES}
  PRIVATE ${GLIB_LIBRARIES})

target_include_directories(
  gstreaming_common
  PRIVATE include
  PRIVATE ${GSTREAMER_INCLUDE_DIRS}
  PRIVATE ${GLIB_INCLUDE_DIRS} SYSTEM
  INTERFACE include)
