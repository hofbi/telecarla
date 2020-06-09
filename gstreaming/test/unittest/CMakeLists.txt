# Library unittests
catkin_add_gtest(
  ${PROJECT_NAME}_unittests main.cpp simple_unit_test.cpp
  rtsp/server/rtsp_server_test.cpp rtsp/client/rtsp_client_test.cpp)

target_link_libraries(${PROJECT_NAME}_unittests rtsp_client rtsp_server)
