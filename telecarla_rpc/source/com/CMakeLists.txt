find_package(rpclib REQUIRED)

add_library(rpc_com src/ros_msg_callback.cpp include/com/ros_msg_callback.h
                    src/rpc_msg_callback.cpp include/com/rpc_msg_callback.h)

target_include_directories(
  rpc_com
  PRIVATE include/com
  PRIVATE ${catkin_INCLUDE_DIRS} SYSTEM
  INTERFACE include/com
  INTERFACE ${catkin_INCLUDE_DIRS} SYSTEM)

target_link_libraries(rpc_com INTERFACE rpclib::rpc pthread)

install(
  TARGETS rpc_com
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
