cmake_minimum_required(VERSION 2.8.3)
project(planner_standalone_optimal)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-diagnostics-color")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

# find all cpp files in currect directory (where CMakeLists.txt is)
file(GLOB SOURCE_FILES FILES_MATCHING PATTERN "src/*.cpp")

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${X11_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}
    ${X11_LIBRARIES}
    ctop
    ctop_util
    ctop_log
    crl
    #crl-gui
    crl-algorithm
    crl-loader
    crl-tsplib
    yaml-cpp
)



