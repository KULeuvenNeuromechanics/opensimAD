cmake_minimum_required(VERSION 3.10)
project(osimad_external LANGUAGES C)
set(OUTPUT_NAME "osimad_external" CACHE STRING "Public library filename")
set(INSTALL_DIR "" CACHE PATH "Install directory")
set(CMAKE_INSTALL_PREFIX "${INSTALL_DIR}" CACHE PATH "Install directory" FORCE)
add_library(osimad_external SHARED foo_jac.c)
set_target_properties(osimad_external PROPERTIES OUTPUT_NAME "${OUTPUT_NAME}")
install(TARGETS osimad_external
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)
