# Projects Settings
cmake_minimum_required (VERSION 3.1)
project (WIP)

#static compile
#set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++")

# add include dir
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

#include matplotlib
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/external/matplotlibcpp")

# find python libraries
find_package(Python 3.7 EXACT COMPONENTS Interpreter Development NumPy REQUIRED)
include_directories(${Python_INCLUDE_DIRS} ${Python_NumPy_INCLUDE_DIRS})

#add glm
add_subdirectory(external/glm)

#add CGAL
find_package(CGAL REQUIRED COMPONENTS Core)

# add executables
add_executable(optimizedGrahamScan optimized_graham_scan.cpp src/input_generators.cpp)
add_executable(grahamScan graham_scan.cpp src/input_generators.cpp)
add_executable(quadtreeScan quadtree_scan.cpp src/input_generators.cpp)
add_executable(quickhull quickhull.cpp src/input_generators.cpp)
add_executable(octreeScan octree_scan.cpp src/input_generators.cpp)

#link main
target_link_libraries(optimizedGrahamScan
        Python::Python
        Python::Module
        Python::NumPy
        )

#link gscan
target_link_libraries(grahamScan CGAL::CGAL CGAL::CGAL_Core)

#link quadscan
target_link_libraries(quadtreeScan CGAL::CGAL CGAL::CGAL_Core)
target_link_libraries(quadtreeScan
        Python::Python
        Python::Module
        Python::NumPy
        )

#link quickhull
target_link_libraries(quickhull CGAL::CGAL CGAL::CGAL_Core)
target_link_libraries(quickhull
        Python::Python
        Python::Module
        Python::NumPy
        )