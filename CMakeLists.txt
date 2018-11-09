cmake_minimum_required(VERSION 3.1)
project(fast_support_reduction)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# libigl

option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_VIEWER            "Use OpenGL viewer"  ON)
find_package(LIBIGL REQUIRED QUIET)

# source files

file(GLOB SRCFILES 
    src/*.h
    src/*.cpp)

# executables

add_executable(bbw bbw.cpp ${SRCFILES})
target_link_libraries(bbw igl::core igl::opengl_glfw)

add_executable(support_reduction support_reduction.cpp ${SRCFILES})
target_link_libraries(support_reduction igl::core igl::opengl_glfw)