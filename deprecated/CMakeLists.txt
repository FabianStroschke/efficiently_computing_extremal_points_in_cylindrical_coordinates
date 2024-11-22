# add include dir
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

#add glm
add_subdirectory(external/glm)

# add executables
add_executable(DEPRECATED_optimizedGrahamScan optimized_graham_scan.cpp src/input_generators.cpp)
add_executable(DEPRECATED_grahamScan graham_scan.cpp src/input_generators.cpp)
add_executable(DEPRECATED_quadtreeScan quadtree_scan.cpp src/input_generators.cpp)
add_executable(DEPRECATED_octreeScan octree_scan.cpp src/input_generators.cpp src/tree_scan_helper.cpp)



#link gscan
target_link_libraries(DEPRECATED_grahamScan CGAL::CGAL CGAL::CGAL_Core)

#link quadscan
target_link_libraries(DEPRECATED_quadtreeScan CGAL::CGAL CGAL::CGAL_Core)

target_link_libraries(DEPRECATED_octreeScan CGAL::CGAL CGAL::CGAL_Core)
