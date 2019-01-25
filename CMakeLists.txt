cmake_minimum_required(VERSION 3.1)
project(fast_support_reduction)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

option(WITH_BBW         "Generate bin for bbw"               OFF)
option(WITH_VISUALIZE   "Geneate bin for visualization"      OFF)
option(WITH_DEBUG       "Enable debugging"                   ON)

if(WITH_DEBUG)
    add_compile_options(-O0)
    add_definitions(-DDEBUG)
    add_definitions(${CMAKE_CXX_FLAGS} "-g")
    add_definitions(${CMAKE_CXX_FLAGS} "-Wall")
else()
    add_compile_options(-O3)
    add_definitions(-DNDEBUG)
    add_definitions(${CMAKE_CXX_FLAGS} "-march=native")
endif()


# libigl

option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_VIEWER            "Use OpenGL viewer"  ON)
option(LIBIGL_WITH_TETGEN            "Use Tetgen"         ON)
find_package(LIBIGL REQUIRED QUIET)

# minitrace

add_definitions(-DMTR_ENABLED)

# Clara 

include_directories(libs/Clara/include)

# source files

file(GLOB SRCFILES 
    src/*.h
    src/*.cpp)

# executables

if(WITH_BBW)
    add_executable(bbw bbw.cpp ${SRCFILES})
    target_link_libraries(bbw igl::core igl::opengl_glfw igl::tetgen)
endif()

add_executable(support_reduction support_reduction.cpp ${SRCFILES})
target_link_libraries(support_reduction igl::core igl::opengl_glfw)

if(WITH_VISUALIZE)
    add_definitions(-DVISUALIZE)
    add_executable(visualize visualize.cpp ${SRCFILES})
    target_link_libraries(visualize igl::core igl::opengl_glfw)
endif()