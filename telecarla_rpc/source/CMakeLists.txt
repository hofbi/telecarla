add_subdirectory(com)
add_subdirectory(data)

add_library(rpc_server STATIC src/telecarla_rpc_server.cpp
                              include/telecarla_rpc_server.h)

target_include_directories(
  rpc_server
  PRIVATE include ${catkin_INCLUDE_DIRS}
  INTERFACE include ${catkin_INCLUDE_DIRS})

target_link_libraries(rpc_server PRIVATE rpc_com rpc_data)

add_library(rpc_client STATIC src/telecarla_rpc_client.cpp
                              include/telecarla_rpc_client.h)

target_include_directories(
  rpc_client
  PRIVATE include ${catkin_INCLUDE_DIRS}
  INTERFACE include ${catkin_INCLUDE_DIRS})

target_link_libraries(rpc_client PRIVATE rpc_com rpc_data)

# Nodes
add_executable(telecarla_rpc_server_node src/telecarla_rpc_server_node.cpp)
target_link_libraries(telecarla_rpc_server_node PRIVATE rpc_server
                                                        ${catkin_LIBRARIES})

add_executable(telecarla_rpc_client_node src/telecarla_rpc_client_node.cpp)
target_link_libraries(telecarla_rpc_client_node PRIVATE rpc_client
                                                        ${catkin_LIBRARIES})
