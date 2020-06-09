find_package(rpclib REQUIRED)

add_library(
  rpc_server STATIC
  src/telecarla_rpc_server.cpp include/telecarla_rpc_server.h
  src/bool_callback.cpp include/bool_callback.h
  src/control_command_callback.cpp include/control_command_callback.h)

target_include_directories(
  rpc_server
  PRIVATE include SYSTEM
  INTERFACE include)

target_link_libraries(
  rpc_server
  PUBLIC rpc_common
  PRIVATE rpclib::rpc pthread)
