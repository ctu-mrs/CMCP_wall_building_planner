cmake_minimum_required(VERSION 2.8.3)
project(planner_standalone_grasp)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

find_package(yaml-cpp REQUIRED)

add_subdirectory(ctop)
add_subdirectory(lib)
add_subdirectory(standalone)
