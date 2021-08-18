find_package(rpclib REQUIRED)

add_library(
  rpc_data
  src/control_commands.cpp include/control_commands.h src/vehicle_status.cpp
  include/vehicle_status.h src/bool.cpp include/bool.h)

target_include_directories(
  rpc_data
  PRIVATE include
  PRIVATE ${catkin_INCLUDE_DIRS} SYSTEM
  INTERFACE include
  INTERFACE ${catkin_INCLUDE_DIRS} SYSTEM)

target_link_libraries(rpc_data INTERFACE rpclib::rpc pthread)
