find_package(SDL2 REQUIRED)
find_package(OpenCV REQUIRED)

add_library(
  gui
  src/gui_parameter.cpp
  include/gui_parameter.h
  include/joystick_parameter.h
  include/sdl_lifecycle.h
  src/sdl_lifecycle.cpp
  include/sdl_event_parser.h
  src/sdl_event_parser.cpp
  include/sdl_gui.h
  src/sdl_gui.cpp
  include/sdl_wheel_controller.h
  src/sdl_wheel_controller.cpp)

target_link_libraries(
  gui
  PUBLIC ${catkin_LIBRARIES} util
  PRIVATE ${OpenCV_LIBRARIES} ${SDL2_LIBRARIES})

target_include_directories(
  gui
  PRIVATE include
  PRIVATE ${catkin_INCLUDE_DIRS} SYSTEM
  INTERFACE include SYSTEM
  INTERFACE ${catkin_INCLUDE_DIRS})

target_compile_features(gui PUBLIC cxx_std_17)

add_dependencies(gui ${catkin_EXPORTED_TARGETS})
