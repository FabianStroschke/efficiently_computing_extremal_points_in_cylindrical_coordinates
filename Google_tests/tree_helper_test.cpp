//
// Created by fabia on 18.06.2023.
//
#include "gtest/gtest.h"
#include "tree_scan_helper.h"
#include "gift_wrapping.h"
#include "input_generators.h"

#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>

//typedefs
typedef CGAL::Polyhedron_3<Kernel> Polyhedron_3;

void testFindBoundary(long seed, long nPoints, int bucketSize){

    //generate input
    auto input = generateInputVec3(nPoints, seed);

    //build octree
    Octree octree(input);
    octree.refine(10, bucketSize);

    //build convex hull
    Polyhedron_3 poly;
    CGAL::convex_hull_3(input.begin(), input.end(), poly);
    long tests_total = 0;
    long tests_failed = 0;
    long tests_complete = 0;
    long tests_nan = 0;

    //iterate over faces
    for(auto it = poly.facets_begin(); it != poly.facets_end(); it++){
        auto face_iterator = *it->facet_begin();

        Kernel::Point_3 origin(0,0,0);
        for(auto &p: input){
            origin = {origin.x()+p.x()/input.size(),origin.y()+p.y()/input.size(),origin.z()+p.z()/input.size()};
        }

        if(face_iterator.face()->is_triangle()){
            tests_total += 3;
            Kernel::Point_3 vertices[3] = {
                    face_iterator.vertex()->point(),
                    face_iterator.next()->vertex()->point(),
                    face_iterator.next()->next()->vertex()->point()
            };
            for (int i = 0; i < 3; ++i) {
                auto resStack = findExtremalPoint(octree, {vertices[(i + 1) % 3], vertices[i]}, BS_RIGHT, origin);
                for (auto res: resStack) {
                    if (res != nullptr) {
                        EXPECT_EQ(*res, vertices[(i + 2) % 3])
                                            << "Wrong solution. At edge: " << "{" << vertices[i] << "},{"
                                            << vertices[(i + 1) % 3] << "} Seed: " << seed << " Size: "
                                            << input.size();
                        if (*res == vertices[(i + 2) % 3]) {
                            tests_complete++;
                        } else {
                            tests_failed++;
                        }
                    } else {
                        EXPECT_TRUE(res) << "No solution found";
                        tests_nan++;
                    }
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
    auto input = generateInputVec3(nPoints, seed);

    //build octree
    Kd_tree kd_tree(input.begin(),input.end(), Kd_tree::Splitter(2));
    kd_tree.build();

    //build convex hull
    Polyhedron_3 poly;
    CGAL::convex_hull_3(input.begin(), input.end(), poly);
    long tests_total = 0;
    long tests_failed = 0;
    long tests_complete = 0;
    long tests_nan = 0;

    Kernel::Point_3 origin(0,0,0);
    for(auto &p: kd_tree){
        origin = {origin.x()+p.x()/kd_tree.size(),origin.y()+p.y()/kd_tree.size(),origin.z()+p.z()/kd_tree.size()};
    }

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
                auto resStack = findExtremalPoint(kd_tree, {vertices[(i + 1) % 3], vertices[i]}, BS_RIGHT, origin);
                for (auto res:resStack) {
                    if(res != nullptr){
                        EXPECT_EQ(*res, vertices[(i+2)%3]) << "Wrong solution. At edge: " << "{" << vertices[i] << "},{" << vertices[(i+1)%3] <<"} Seed: "<< seed << " Size: " <<kd_tree.size();
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

void testFindBoundaryGW(long seed, long nPoints){

    //generate input
    auto input = generateInputVec3(nPoints, seed);

    //build convex hull
    Polyhedron_3 poly;
    CGAL::convex_hull_3(input.begin(), input.end(), poly);
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
                auto resStack = findExtremalPoint(input, {vertices[(i + 1) % 3], vertices[i]},
                                                  {vertices[0], vertices[1], vertices[2]});
                for (auto res:resStack) {
                    if(res != nullptr){
                        EXPECT_EQ(*res, vertices[(i+2)%3]) << "Wrong solution. At edge: " << "{" << vertices[i] << "},{" << vertices[(i+1)%3] <<"} Seed: "<< seed << " Size: " <<input.size();
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

TEST(FindBoundaryKD, C10M){
    /*for(int i = 0; i<100000; i++){
        testFindBoundaryKD(i,10);
    }*/
    testFindBoundaryKD(790,10);
}

TEST(FindBoundaryGW, C500){
    testFindBoundaryGW(1404,500);
}TEST(FindBoundaryGW, C5000){
    testFindBoundaryGW(1404,5000);
}
TEST(FindBoundaryGW, C50000){
    testFindBoundaryGW(1404,50000);
}
TEST(FindBoundaryGW, C500000){
    testFindBoundaryGW(1404,500000);
}
