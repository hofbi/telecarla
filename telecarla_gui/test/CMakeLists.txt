configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/config/one_camera_config.json
  ${CMAKE_CURRENT_BINARY_DIR}/gui/config/one_camera_config.json COPYONLY)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/config/two_camera_config.json
  ${CMAKE_CURRENT_BINARY_DIR}/gui/config/two_camera_config.json COPYONLY)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/config/three_camera_config.json
  ${CMAKE_CURRENT_BINARY_DIR}/gui/config/three_camera_config.json COPYONLY)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/config/vehicle_status_config.json
  ${CMAKE_CURRENT_BINARY_DIR}/gui/config/vehicle_status_config.json COPYONLY)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/config/static_text_config.json
  ${CMAKE_CURRENT_BINARY_DIR}/gui/config/static_text_config.json COPYONLY)

# Library unittests
catkin_add_gtest(
  ${PROJECT_NAME}_unittests main.cpp simple_test.cpp gui/gui_parameter_test.cpp
  util/linear_function_test.cpp util/steer_cache_test.cpp)

target_link_libraries(${PROJECT_NAME}_unittests telecarla_gui)

# ROS Tests

# Roslaunch Checks
roslaunch_add_file_check(${PROJECT_SOURCE_DIR}/launch DEPENDENCIES
                         telecarla_gui_node)
