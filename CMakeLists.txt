# Projects Settings
cmake_minimum_required(VERSION 3.1)
project(WIP)

#static compile
#set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++")

# add include dir
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

#add CGAL
find_package(CGAL REQUIRED COMPONENTS Core)

# add executables
add_executable(quickhull quickhull.cpp src/input_generators.cpp)
add_executable(octreeWrap octree_gift_wrapping.cpp src/input_generators.cpp src/tree_scan_helper.cpp src/octree_wrap.cpp)
add_executable(KDWrap kd_wrap_demo.cpp src/input_generators.cpp src/tree_scan_helper.cpp src/kd_wrap.cpp)
add_executable(demo src/ExecutionWrapper.cpp src/input_generators.cpp src/tree_scan_helper.cpp src/octree_wrap.cpp src/kd_wrap.cpp src/gift_wrapping.cpp)
add_executable(compare compare.cpp src/input_generators.cpp src/tree_scan_helper.cpp src/octree_wrap.cpp src/kd_wrap.cpp src/gift_wrapping.cpp)

#link quickhull
target_link_libraries(quickhull CGAL::CGAL CGAL::CGAL_Core)

target_link_libraries(octreeWrap CGAL::CGAL CGAL::CGAL_Core)

target_link_libraries(KDWrap CGAL::CGAL CGAL::CGAL_Core)


target_link_libraries(demo CGAL::CGAL CGAL::CGAL_Core)
target_link_libraries(compare CGAL::CGAL CGAL::CGAL_Core)

add_subdirectory(Google_tests)
add_subdirectory(deprecated)