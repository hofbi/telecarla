# Unit Tests
add_subdirectory(unittest)

# ROS Tests add_subdirectory(rostest)

# Roslaunch Checks
roslaunch_add_file_check(${PROJECT_SOURCE_DIR}/launch)
roslaunch_add_file_check(rostest/launch)
