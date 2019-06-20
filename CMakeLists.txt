cmake_minimum_required(VERSION 3.1)
project(fast_support_reduction)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

option(WITH_VISUALIZE   "Geneate bin for visualization"      OFF)
option(WITH_DEBUG       "Enable debugging"                   OFF)

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



# SET(OpenMP_C_FLAGS       "-Xpreprocessor -fopenmp -I/usr/local/opt/libomp/include")
# SET(OpenMP_C_LIB_NAMES   "omp")
# SET(OpenMP_CXX_FLAGS     "-Xpreprocessor -fopenmp -I/usr/local/opt/libomp/include")
# SET(OpenMP_CXX_LIB_NAMES "omp")
# SET(OpenMP_omp_LIBRARY   "/usr/local/opt/libomp/lib/libomp.dylib")
# find_package(OpenMP REQUIRED)


# libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_VIEWER            "Use OpenGL viewer"  ON)
option(LIBIGL_WITH_TETGEN            "Use Tetgen"         ON)
option(LIBIGL_WITH_TRIANGLE          "Use Triangle"       ON)
option(LIBIGL_WITH_PNG               "Use PNG"            ON)
option(LIBIGL_WITH_CGAL              "Use CGAL"           ON)
find_package(LIBIGL REQUIRED QUIET)


# cgal

find_package(CGAL REQUIRED)
include(${CGAL_USE_FILE})
set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE CACHE BOOL "Force CGAL to maintain CMAKE flags")

# minitrace

add_definitions(-DMTR_ENABLED)

# stan
include_directories(libs/math) # header-only
include_directories(libs/math/lib/eigen_3.3.3) # not sure why this works...
include_directories(libs/math/lib/boost_1.69.0)
include_directories(libs/math/lib/sundials_4.1.0)
include_directories(libs/math/lib/cpplint_4.45)
include_directories(libs/math/lib/gtest_1.8.1)
include_directories(libs/math/lib/opencl_1.2.8)


# source files
include_directories(src)
include_directories(src/opengl)
include_directories(src/utils)
include_directories(src/stress)
include_directories(src/weights)

file(GLOB SRCFILES 
    src/*.h
    src/*.cpp
    src/opengl/*h
    src/opengl/*cpp
    src/utils/*h
    src/utils/*cpp
    src/weights/*h
    src/weights/*cpp
    src/stress/*h
    src/stress/*cpp)

# executables

add_executable(shape_opt shape_opt.cpp ${SRCFILES})
target_link_libraries(shape_opt igl::core igl::opengl_glfw igl::png igl::tetgen)

# add_executable(support_reduction support_reduction.cpp ${SRCFILES})
# target_link_libraries(support_reduction igl::core igl::opengl_glfw igl::png igl::tetgen)

# add_executable(2d_bbw 2d_bbw.cpp ${SRCFILES})
# target_link_libraries(2d_bbw igl::core igl::opengl_glfw igl::png igl::tetgen)

# add_executable(gl gl.cpp ${SRCFILES})
# target_link_libraries(gl igl::core igl::opengl_glfw igl::png igl::tetgen)

# add_executable(stress_visualizer stress_visualizer.cpp ${SRCFILES})
# target_link_libraries(stress_visualizer igl::core igl::opengl_glfw igl::png igl::tetgen)

# add_executable(mesh_file_generator mesh_file_generator.cpp ${SRCFILES})
# target_link_libraries(mesh_file_generator igl::core igl::opengl_glfw igl::tetgen igl::png)


if(WITH_VISUALIZE)
    add_definitions(-DVISUALIZE)
    add_executable(visualize visualize.cpp ${SRCFILES})
    target_link_libraries(visualize igl::core igl::opengl_glfw)
endif()