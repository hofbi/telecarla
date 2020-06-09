add_library(util src/linear_function.cpp include/linear_function.h
                 src/steer_cache.cpp include/steer_cache.h)

target_include_directories(
  util
  PRIVATE include SYSTEM
  INTERFACE include)

target_compile_features(util PUBLIC cxx_std_17)
