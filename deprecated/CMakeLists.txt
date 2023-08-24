# add include dir
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

#include matplotlib
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/external/matplotlibcpp")

#add glm
add_subdirectory(external/glm)

# add executables
add_executable(DEPRECATED_optimizedGrahamScan optimized_graham_scan.cpp src/input_generators.cpp)
add_executable(DEPRECATED_grahamScan graham_scan.cpp src/input_generators.cpp)
add_executable(DEPRECATED_quadtreeScan quadtree_scan.cpp src/input_generators.cpp)
add_executable(DEPRECATED_octreeScan octree_scan.cpp src/input_generators.cpp src/tree_scan_helper.cpp)

#link main
target_link_libraries(DEPRECATED_optimizedGrahamScan
        Python::Python
        Python::Module
        Python::NumPy
        )

#link gscan
target_link_libraries(DEPRECATED_grahamScan CGAL::CGAL CGAL::CGAL_Core)

#link quadscan
target_link_libraries(DEPRECATED_quadtreeScan CGAL::CGAL CGAL::CGAL_Core)
target_link_libraries(DEPRECATED_quadtreeScan
        Python::Python
        Python::Module
        Python::NumPy
        )
target_link_libraries(DEPRECATED_octreeScan CGAL::CGAL CGAL::CGAL_Core)
target_link_libraries(DEPRECATED_octreeScan
        Python::Python
        Python::Module
        Python::NumPy
        )