cmake_minimum_required(VERSION 2.8.3)
project(ctop)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Og")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-diagnostics-color")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

set(CTOP_NAMESPACE ctop)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

#cplex define for modern compilators
#otherwise it will use old 90s headers like #include <iostream.h> instead <iostream>

if(UNIX AND NOT APPLE)
    set(LINUX TRUE)
endif()

find_package(CPLEX)
message(STATUS "CPLEX_FOUND: ${CPLEX_FOUND}")
message(STATUS "CPLEX_INCLUDE_DIR: ${CPLEX_INCLUDE_DIR}")
message(STATUS "CPLEX_LIBRARIES: ${CPLEX_LIBRARIES}")
message("")

if(NOT CPLEX_FOUND)
    return()
endif()

file(GLOB_RECURSE SOURCE_FILES FILES_MATCHING PATTERN "./src/*.c*")
file(GLOB_RECURSE HEADER_FILES FILES_MATCHING PATTERN "./src/*.h*")

execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/include)
execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${PROJECT_SOURCE_DIR}/src ${CMAKE_CURRENT_BINARY_DIR}/include/ctop)

message(STATUS "ctop source files: " ${SOURCE_FILES} ${HEADER_FILES})

add_library(${PROJECT_NAME} STATIC
    ${SOURCE_FILES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_BINARY_DIR}/include
    ${Boost_INCLUDE_DIRS}
)

# cplex linking
# https://www.ibm.com/support/knowledgecenter/SSSA5P_12.7.0/ilog.odms.cplex.help/CPLEX/GettingStarted/topics/set_up/GNU_Linux.html
# cplex library libiocplex.a

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${Boost_LIBRARIES}
    ${CPLEX_LIBRARIES}
    Threads::Threads
    fmt
    ctop_common
    ctop_util
    # FindCPLEX.cmake
    # cplex-library - the CPLEX library
    # cplex-concert - the Concert library
    # ilocplex - the IloCplex library
    # cplex-cp - the CP Optimizer library
    cplex-library
    cplex-concert
    ilocplex
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
    IL_STD=1
)



