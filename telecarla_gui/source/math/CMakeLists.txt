add_library(math src/linear_function.cpp include/linear_function.h)

target_include_directories(
  math
  PRIVATE include SYSTEM
  INTERFACE include)

target_compile_features(math PUBLIC cxx_std_17)
