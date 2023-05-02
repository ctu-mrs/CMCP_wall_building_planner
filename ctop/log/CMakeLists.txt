cmake_minimum_required(VERSION 3.5)
project(ctop_log)

#Create base library
file(GLOB_RECURSE SOURCE_FILES FILES_MATCHING PATTERN "src/*")

add_library(ctop_log ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(ctop_log fmt)
target_include_directories(ctop_log PUBLIC src)
target_compile_definitions(ctop_log PRIVATE CTOP_LOG_ROOT_PROJECT_PATH=\"${PROJECT_ROOT_DIR}/\")
