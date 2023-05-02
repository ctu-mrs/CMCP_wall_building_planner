# external libraries source code

# modern formatting library
add_subdirectory(fmt)
add_subdirectory(eigen)

add_subdirectory(comrob/crl)
# fixup combrob library include directories
if (TARGET crl)
    target_include_directories(crl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/comrob/crl/crl)
endif()
if (TARGET crl-gui)
    target_include_directories(crl-gui PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/comrob/crl/crl.gui)
endif()
if (TARGET crl-algorithm)
    target_include_directories(crl-algorithm PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/comrob/crl/crl.algorithm)
endif()
if (TARGET crl-loader)
    target_include_directories(crl-loader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/comrob/crl/crl.loader)
endif()
if (TARGET crl-tsplib)
    target_include_directories(crl-tsplib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/comrob/crl/crl.tsplib)
endif()



