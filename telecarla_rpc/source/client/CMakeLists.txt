find_package(rpclib REQUIRED)

add_library(
  rpc_client STATIC
  src/telecarla_rpc_client.cpp include/telecarla_rpc_client.h
  src/bool_msg_callback.cpp include/bool_msg_callback.h
  src/control_command_msg_callback.cpp include/control_command_msg_callback.h)

target_include_directories(
  rpc_client
  PRIVATE include SYSTEM
  INTERFACE include SYSTEM
  INTERFACE ${catkin_INCLUDE_DIRS})

target_link_libraries(
  rpc_client
  PRIVATE rpc_common
  PRIVATE rpclib::rpc pthread)
