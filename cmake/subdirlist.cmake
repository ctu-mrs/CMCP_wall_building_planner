MACRO(SUBDIRLIST result curdir)
    FILE(GLOB children RELATIVE ${curdir} "${curdir}/*")
    SET(dirlist "")
    FOREACH(child ${children})
        IF(IS_DIRECTORY ${curdir}/${child})
            LIST(APPEND dirlist ${child})
        ENDIF()
    ENDFOREACH()
    SET(${result} ${dirlist})
ENDMACRO()

#usage
#SUBDIRLIST(SUBDIRS ${PROGRAMS_PATH})
#
#FOREACH(subdir ${SUBDIRS})
#    message("found "${subdir})
#ENDFOREACH()