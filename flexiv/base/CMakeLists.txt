cmake_minimum_required(VERSION 3.16.3)

set(CMAKE_INSTALL_PREFIX "/home/whoami/rdk_install" CACHE PATH "Installation Directory" FORCE)

option(INSTALL_PYTHON_RDK "Install Python RDK" ON)

find_package(flexiv_rdk REQUIRED)
# add_executable(demo ../test_control.cpp)
add_executable(sine sine_control_demo.cpp)
add_executable(main main.cpp)
add_executable(gripper gripper_test.cpp)
# target_link_libraries(demo flexiv::flexiv_rdk)
target_link_libraries(sine flexiv::flexiv_rdk)
target_link_libraries(main flexiv::flexiv_rdk)
target_link_libraries(gripper flexiv::flexiv_rdk)
