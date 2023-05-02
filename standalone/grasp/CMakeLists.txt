cmake_minimum_required(VERSION 2.8.3)
project(planner_standalone_grasp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-diagnostics-color")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

# find all cpp files in currect directory (where CMakeLists.txt is)
file(GLOB SOURCE_FILES FILES_MATCHING PATTERN "./src/*.cpp")

include_directories(src)
add_executable(${PROJECT_NAME} main.cpp ${SOURCE_FILES})
add_executable(${PROJECT_NAME}_snap main_snap.cpp ${SOURCE_FILES})
add_executable(${PROJECT_NAME}_multi main_multicore.cpp ${SOURCE_FILES})

set(EXE_LIBS
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

target_link_libraries(${PROJECT_NAME} ${EXE_LIBS})
target_link_libraries(${PROJECT_NAME}_snap ${EXE_LIBS})
target_link_libraries(${PROJECT_NAME}_multi ${EXE_LIBS})


