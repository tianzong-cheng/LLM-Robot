cmake_minimum_required(VERSION 3.16.3)
project(flexiv)

# Show verbose build info
SET(CMAKE_VERBOSE_MAKEFILE ON)

# Minumum example list for all OS
set(EXAMPLE_LIST
  home
  run
)

# Find flexiv_rdk INTERFACE library
find_package(flexiv_rdk REQUIRED)

# Build all selected examples
foreach(example ${EXAMPLE_LIST})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} flexiv::flexiv_rdk)
  # C++14 required
  set_target_properties(${example} PROPERTIES 
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON)
endforeach()
