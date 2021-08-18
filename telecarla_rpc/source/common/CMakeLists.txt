find_package(rpclib REQUIRED)

add_library(rpc_common src/ros_msg_callback.cpp include/ros_msg_callback.h
                       src/rpc_msg_callback.cpp include/rpc_msg_callback.h)

target_include_directories(
  rpc_common
  PRIVATE include
  PRIVATE ${catkin_INCLUDE_DIRS} SYSTEM
  INTERFACE include
  INTERFACE ${catkin_INCLUDE_DIRS} SYSTEM)

target_link_libraries(rpc_common INTERFACE rpclib::rpc pthread)
