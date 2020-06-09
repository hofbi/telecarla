add_subdirectory(common)
add_subdirectory(server)
add_subdirectory(client)

# Nodes
add_executable(telecarla_rpc_server_node src/telecarla_rpc_server_node.cpp)
target_link_libraries(telecarla_rpc_server_node PRIVATE rpc_server
                                                        ${catkin_LIBRARIES})

add_executable(telecarla_rpc_client_node src/telecarla_rpc_client_node.cpp)
target_link_libraries(telecarla_rpc_client_node PRIVATE rpc_client
                                                        ${catkin_LIBRARIES})
