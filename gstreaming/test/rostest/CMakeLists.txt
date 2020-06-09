find_package(rostest REQUIRED)

add_rostest_gtest(${PROJECT_NAME}_rostest_testrunner launch/gtest.test main.cpp
                  simple_ros_test.cpp)

target_link_libraries(${PROJECT_NAME}_rostest_testrunner ${catkin_LIBRARIES})

target_include_directories(${PROJECT_NAME}_rostest_testrunner
                           PRIVATE ${catkin_INCLUDE_DIRS})

add_rostest(launch/clienttest_no_server_input.test DEPENDENCIES
            rtsp_server_node rtsp_client_node)
add_rostest(launch/clienttest_with_server_input.test DEPENDENCIES
            rtsp_server_node rtsp_client_node)
