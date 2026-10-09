cmake_minimum_required(VERSION 3.10)
project(osimad_graph LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 11)
add_executable(osimad_graph graph.cpp)
set(SDK_DIR "" CACHE PATH "OpenSimAD SDK directory")
target_include_directories(osimad_graph PUBLIC "${SDK_DIR}/include" "${SDK_DIR}/Simbody/include")
target_compile_definitions(osimad_graph PUBLIC SimTK_REAL_IS_ADOUBLE)
target_link_libraries(osimad_graph PUBLIC
    "${SDK_DIR}/lib/osimCommon.lib" "${SDK_DIR}/lib/osimSimulation.lib"
    "${SDK_DIR}/Simbody/lib/SimTKcommon_recorder.lib"
    "${SDK_DIR}/Simbody/lib/SimTKmath_recorder.lib"
    "${SDK_DIR}/Simbody/lib/SimTKsimbody_recorder.lib")
