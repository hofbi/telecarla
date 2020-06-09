# Libraries
add_subdirectory(common)
add_subdirectory(rtsp)

# Executeables
add_executable(
  rtsp_server_node src/gstreaming_server_node.cpp include/gstreaming_server.h
                   src/gstreaming_server.cpp)

add_executable(
  rtsp_client_node src/gstreaming_client_node.cpp include/gstreaming_client.h
                   src/gstreaming_client.cpp)

target_link_libraries(
  rtsp_server_node
  PRIVATE ${catkin_LIBRARIES} ${GSTREAMER_NET_LIBRARIES}
  PRIVATE rtsp_server gstreaming_common)
target_include_directories(rtsp_server_node PRIVATE ${catkin_INCLUDE_DIRS}
                                                    include)
add_dependencies(rtsp_server_node ${PROJECT_NAME}_gencfg)

target_link_libraries(
  rtsp_client_node
  PRIVATE ${catkin_LIBRARIES} ${GSTREAMER_NET_LIBRARIES}
  PRIVATE rtsp_client gstreaming_common)
target_include_directories(rtsp_client_node PRIVATE ${catkin_INCLUDE_DIRS}
                                                    include)
target_compile_features(rtsp_client_node PRIVATE cxx_std_14)
