cmake_minimum_required(VERSION 2.8.3)
project(ctop_common)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Og")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-diagnostics-color")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

set(CTOP_NAMESPACE ctop)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)
# cplex define for modern compilations
# otherwise it will use old 90s headers like #include <iostream.h> instead <iostream>

file(GLOB_RECURSE SOURCE_FILES FILES_MATCHING PATTERN "./src/*.c*")
file(GLOB_RECURSE HEADER_FILES FILES_MATCHING PATTERN "./src/*.h*")

message(STATUS "ctop source files: " ${SOURCE_FILES} ${HEADER_FILES})

add_library(${PROJECT_NAME} STATIC
    ${SOURCE_FILES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${Boost_LIBRARIES}
    Threads::Threads
    fmt
    crl
    ctop_log
    eigen
)



