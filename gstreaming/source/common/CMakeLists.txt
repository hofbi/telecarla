add_library(gstreaming_common include/gst_lifecycle.h src/gst_lifecycle.cpp
                              include/pipeline_state.h src/pipeline_state.cpp)

target_link_libraries(gstreaming_common PRIVATE ${GSTREAMER_LIBRARIES}
                                                ${GLIB_LIBRARIES})

target_include_directories(
  gstreaming_common
  PRIVATE include ${GSTREAMER_INCLUDE_DIRS} ${GLIB_INCLUDE_DIRS}
  INTERFACE include)
