# Projects Settings
cmake_minimum_required (VERSION 3.1)
project (WIP)

#static compile
#set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++")

# add include dir
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

# find python libraries
find_package(Python 3.7 EXACT COMPONENTS Interpreter Development NumPy REQUIRED)
include_directories(${Python_INCLUDE_DIRS} ${Python_NumPy_INCLUDE_DIRS})

# populate matplotlib repository
include(FetchContent)
FetchContent_Declare(
        matplotlib
        GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
        GIT_TAG        f23347fca25219d1c42cbb91608b5556814bf572
)
FetchContent_GetProperties(matplotlib)
if(NOT matplotlib_POPULATED)
    FetchContent_Populate(matplotlib)
endif()
include_directories(SYSTEM ${matplotlib_SOURCE_DIR})

#add glm
add_subdirectory(external/glm)

#add CGAL
find_package(CGAL REQUIRED COMPONENTS Core)

# add executables
add_executable(main main.cpp src/input_generators.cpp src/modifiedGrahamScan.cpp)
add_executable(gscan graham_scan.cpp src/input_generators.cpp)
add_executable(quadscan quadtree_scan.cpp src/input_generators.cpp)
add_executable(quickhull quickhull.cpp src/input_generators.cpp)

#link main
target_link_libraries(main
        Python::Python
        Python::Module
        Python::NumPy
        )

#link gscan
target_link_libraries(gscan CGAL::CGAL CGAL::CGAL_Core)

#link quadscan
target_link_libraries(quadscan CGAL::CGAL CGAL::CGAL_Core)
target_link_libraries(quadscan
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