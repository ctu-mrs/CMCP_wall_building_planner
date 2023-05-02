cmake_minimum_required(VERSION 3.5)
project(ctop_util)

#Create base library
file(GLOB_RECURSE SOURCE_FILES FILES_MATCHING PATTERN "src/*")

add_library(ctop_util ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(ctop_util
        ctop_log)
target_include_directories(ctop_util PUBLIC src)
target_compile_definitions(ctop_util PUBLIC CTOP_UTIL_ROOT_PROJECT_PATH=\"${PROJECT_ROOT_DIR}/\")
