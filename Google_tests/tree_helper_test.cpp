//
// Created by fabia on 18.06.2023.
//
#include "gtest/gtest.h"
#include "tree_scan_helper.h"
#include "input_generators.h"

#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>

//typedefs
typedef CGAL::Polyhedron_3<Kernel> Polyhedron_3;

void testFindBoundary(long seed, long nPoints, int bucketSize){

    //generate input
    auto input = generateInputVec3(nPoints, seed, FPL_CONVEXHULL);

    //build octree
    Octree octree(input.pointCloud);
    octree.refine(10, bucketSize);

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
                    EXPECT_EQ(*res, vertices[(i+2)%3]) << "Wrong solution. At edge: " << "{" << vertices[i] << "},{" << vertices[(i+1)%3] <<"}";
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

void testFindBoundaryKD(long seed, long nPoints){

    //generate input
    auto input = generateInputVec3(nPoints, seed, FPL_CONVEXHULL);

    //build octree
    Kd_tree kd_tree(input.pointCloud.begin(),input.pointCloud.end(), Kd_tree::Splitter(1));
    kd_tree.build();

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
                auto res = findBoundaryPoint(kd_tree, {vertices[i],vertices[(i+1)%3]},BS_LEFT);
                if(res != nullptr){
                    EXPECT_EQ(*res, vertices[(i+2)%3]) << "Wrong solution. At edge: " << "{" << vertices[i] << "},{" << vertices[(i+1)%3] <<"} Seed: "<< seed;
                    if(*res == vertices[(i+2)%3]){
                        tests_complete++;
                    }else{
                        tests_failed++;
                        /*Kernel::Plane_3 p1(vertices[i],vertices[(i+1)%3],vertices[(i+2)%3]);
                        Kernel::Plane_3 p2(vertices[i],vertices[(i+1)%3],*res);
                        std::cout << "Angle:     " << acos((p1.orthogonal_vector()*p2.orthogonal_vector())
                            /(sqrt(p1.orthogonal_vector().squared_length())*sqrt(p2.orthogonal_vector().squared_length())));*/

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
    if(tests_failed){
        std::cout << "Total Test:     " << tests_total
                  << "\nTests Completed:" << tests_complete
                  << "\nTests Failed:   " << tests_failed
                  << "\nTests NAN:      " << tests_nan;
    }
}

TEST(FindBoundary, C5B1){
    testFindBoundary(1404,5,1);
}
TEST(FindBoundary, C5B5){
    testFindBoundary(1404,5,5);
}

TEST(FindBoundary, C50B1){
    testFindBoundary(1404,50,1);
}
TEST(FindBoundary, C50B5){
    testFindBoundary(1404,50,5);
}
TEST(FindBoundary, C50B15){
    testFindBoundary(1404,50,15);
}
TEST(FindBoundary, C50B50){
    testFindBoundary(1404,50,50);
}

TEST(FindBoundary, C500B1){
    testFindBoundary(1404,500,1);
}
TEST(FindBoundary, C500B5){
    testFindBoundary(1404,500,5);
}
TEST(FindBoundary, C500B15){
    testFindBoundary(1404,500,15);
}
TEST(FindBoundary, C500B50){
    testFindBoundary(1404,500,50);
}

TEST(FindBoundary, C5000B1){
    testFindBoundary(1404,500,1);
}
TEST(FindBoundary, C5000B5){
    testFindBoundary(1404,500,5);
}
TEST(FindBoundary, C5000B15){
    testFindBoundary(1404,500,15);
}
TEST(FindBoundary, C5000B50){
    testFindBoundary(1404,500,50);
}


TEST(FindBoundary, C50000B1){
    testFindBoundary(1404,500,1);
}
TEST(FindBoundary, C50000B5){
    testFindBoundary(1404,500,5);
}
TEST(FindBoundary, C50000B15){
    testFindBoundary(1404,500,15);
}
TEST(FindBoundary, C50000B50){
    testFindBoundary(1404,500,50);
}
TEST(FindBoundaryKD, C500){
    testFindBoundaryKD(1404,500);
}TEST(FindBoundaryKD, C5000){
    testFindBoundaryKD(1404,5000);
}
TEST(FindBoundaryKD, C50000){
    testFindBoundaryKD(1404,50000);
}
TEST(FindBoundaryKD, C500000){
    testFindBoundaryKD(1404,500000);
}

TEST(FindBoundaryKD, C50M){
    for(int i = 0; i<1000000; i++){
        testFindBoundaryKD(i,10);
    }
}

