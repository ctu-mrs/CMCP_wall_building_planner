cmake_minimum_required(VERSION 2.8.3)
project(planner_standalone_plan2tex)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-diagnostics-color")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

# find all cpp files in currect directory (where CMakeLists.txt is)
file(GLOB SOURCE_FILES FILES_MATCHING PATTERN "./src/*.cpp")

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME}
    ctop_common
    ctop_log
    ctop_util

    eigen
    crl
    crl-algorithm
    crl-loader
    crl-tsplib
    yaml-cpp
)



