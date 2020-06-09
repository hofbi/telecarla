add_subdirectory(util)
add_subdirectory(gui)

add_library(
  telecarla_gui
  src/telecarla_gui.cpp include/telecarla_gui.h src/image_callback.cpp
  include/image_callback.h src/status_callback.cpp include/status_callback.h)

target_link_libraries(telecarla_gui PUBLIC gui)

target_include_directories(
  telecarla_gui
  PRIVATE include SYSTEM
  INTERFACE include)

# Executeables
add_executable(telecarla_gui_node src/telecarla_gui_node.cpp)

target_link_libraries(telecarla_gui_node PRIVATE telecarla_gui)
