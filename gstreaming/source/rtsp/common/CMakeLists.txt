add_library(rtsp_common include/rtsp_state.h src/rtsp_state.cpp)

target_include_directories(
  rtsp_common
  PRIVATE include SYSTEM
  INTERFACE include)
