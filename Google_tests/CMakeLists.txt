project(Google_tests)
add_subdirectory(lib)
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

# adding the Google_Tests_run target
add_executable(Google_Tests_run octree_helper_test.cpp octree_wrap_test.cpp ../src/input_generators.cpp ../src/octree_scan_helper.cpp ../src/octree_wrap.cpp)

target_link_libraries(Google_Tests_run CGAL::CGAL CGAL::CGAL_Core)

target_link_libraries(Google_Tests_run gtest gtest_main)