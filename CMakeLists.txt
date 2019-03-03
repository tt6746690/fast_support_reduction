cmake_minimum_required(VERSION 3.1)
project(fast_support_reduction)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
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


# # openmp 
# SET(OpenMP_C_FLAGS       "-Xpreprocessor -fopenmp -I/usr/local/opt/libomp/include")
# SET(OpenMP_C_LIB_NAMES   "omp")
# SET(OpenMP_CXX_FLAGS     "-Xpreprocessor -fopenmp -I/usr/local/opt/libomp/include")
# SET(OpenMP_CXX_LIB_NAMES "omp")
# SET(OpenMP_omp_LIBRARY   "/usr/local/opt/libomp/lib/libomp.dylib")

find_package(OpenMP REQUIRED)

# libigl

option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_VIEWER            "Use OpenGL viewer"  ON)
option(LIBIGL_WITH_TETGEN            "Use Tetgen"         ON)
option(LIBIGL_WITH_PNG               "Use PNG"            ON)
find_package(LIBIGL REQUIRED QUIET)

# cgal

find_package(CGAL REQUIRED)
include(${CGAL_USE_FILE})
set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE CACHE BOOL "Force CGAL to maintain CMAKE flags")

# minitrace

add_definitions(-DMTR_ENABLED)

# Clara 

include_directories(libs/Clara/include)

# source files

include_directories(src)
include_directories(src/opengl)
include_directories(src/utils)
include_directories(src/stress)

file(GLOB SRCFILES 
    src/*.h
    src/*.cpp
    src/opengl/*h
    src/opengl/*cpp
    src/utils/*h
    src/utils/*cpp
    src/stress/*h
    src/stress/*cpp)

# executables

if(WITH_BBW)
    add_executable(bbw bbw.cpp ${SRCFILES})
    target_link_libraries(bbw igl::core igl::opengl_glfw igl::tetgen)
endif()

add_executable(support_reduction support_reduction.cpp ${SRCFILES})
target_link_libraries(support_reduction igl::core igl::opengl_glfw igl::png)
target_link_libraries(support_reduction OpenMP::OpenMP_CXX)

# add_executable(gl gl.cpp ${SRCFILES})
# target_link_libraries(gl igl::core igl::opengl_glfw igl::png)

# add_executable(stress_visualizer stress_visualizer.cpp ${SRCFILES})
# target_link_libraries(stress_visualizer igl::core igl::opengl_glfw igl::png)

if(WITH_VISUALIZE)
    add_definitions(-DVISUALIZE)
    add_executable(visualize visualize.cpp ${SRCFILES})
    target_link_libraries(visualize igl::core igl::opengl_glfw)
endif()