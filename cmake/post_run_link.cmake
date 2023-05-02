function(post_run_link tar)
    if (LINUX)
        add_custom_command(TARGET ${tar} POST_BUILD
                COMMAND /bin/bash
                ${CMAKE_CUSTOM_BASE_SOURCE_DIR}/post_run/linux/run_ldd.sh ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${tar})
    endif (LINUX)
    if(APPLE)
        add_custom_command(TARGET ${tar} POST_BUILD
                COMMAND /bin/bash
                ${CMAKE_CUSTOM_BASE_SOURCE_DIR}/post_run/mac/run_otool.sh ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${tar})
    endif(APPLE)
endfunction()