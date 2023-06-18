//
// Created by fabia on 18.06.2023.
//
#include "gtest/gtest.h"
#include "octree_scan_helper.h"
#include "input_generators.h"

#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>

//typedefs
typedef CGAL::Polyhedron_3<Kernel> Polyhedron_3;

TEST(FindBoundaryTestSuite, ConvexHull){

    //generate input
    auto input = generateInputVec3(500, 1404, FPL_CONVEXHULL);

    //build octree
    Octree octree(input.pointCloud);
    octree.refine(10, 5);

    //build convex hull
    Polyhedron_3 poly;
    CGAL::convex_hull_3(input.pointCloud.begin(), input.pointCloud.end(), poly);
    long tests_total = 0;
    long tests_failed = 0;
    long tests_complete = 0;
    long tests_nan = 0;

    //iterate over faces
    for(auto it = poly.facets_begin(); it != poly.facets_end(); it++){
        auto face_iterator = *it->facet_begin();
        if(face_iterator.face()->is_triangle()){
            tests_total += 3;
            Kernel::Point_3 vertices[3] = {
                    face_iterator.vertex()->point(),
                    face_iterator.next()->vertex()->point(),
                    face_iterator.next()->next()->vertex()->point()
            };
            for (int i = 0; i < 3; ++i) {
                auto res = findBoundaryPoint(octree, {vertices[i],vertices[(i+1)%3]},BS_LEFT);
                if(res != nullptr){
                    EXPECT_EQ(*res, vertices[(i+2)%3]) << "Wrong solution.";
                    if(*res == vertices[(i+2)%3]){
                        tests_complete++;
                    }else{
                        tests_failed++;
                    }
                }else{
                    EXPECT_TRUE(res) << "No solution found";
                    tests_nan++;
                }
            }
        }else{
            EXPECT_TRUE(face_iterator.face()->is_triangle()) << "Face has more than 3 Vertices.";
        }
    }
    std::cout << "Total Test:     " << tests_total
            << "\nTests Completed:" << tests_complete
            << "\nTests Failed:   " << tests_failed
            << "\nTests NAN:      " << tests_nan;
}

