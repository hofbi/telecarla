add_library(rpc_common src/control_commands.cpp include/control_commands.h
                       src/vehicle_status.cpp include/vehicle_status.h)

target_include_directories(
  rpc_common
  PRIVATE include
  PRIVATE ${catkin_INCLUDE_DIRS} SYSTEM
  INTERFACE include SYSTEM
  INTERFACE ${catkin_INCLUDE_DIRS})
