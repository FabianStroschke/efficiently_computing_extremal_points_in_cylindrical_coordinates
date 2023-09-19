//
// Created by fabia on 18.06.2023.
//
#include "gtest/gtest.h"
#include "tree_scan_helper.h"
#include "input_generators.h"
#include "matplot_helper.h"

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
    auto input = generateInputVec3(nPoints, seed);

    //build octree
    Kd_tree kd_tree(input.begin(),input.end(), Kd_tree::Splitter(1));
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
                auto res = findBoundaryPoint(kd_tree, {vertices[(i + 1) % 3],vertices[i]}, BS_RIGHT, origin);
                if(res != nullptr){
                    EXPECT_EQ(*res, vertices[(i+2)%3]) << "Wrong solution. At edge: " << "{" << vertices[i] << "},{" << vertices[(i+1)%3] <<"} Seed: "<< seed << " Size: " <<kd_tree.size();
                    if(*res == vertices[(i+2)%3]){
                        tests_complete++;
                    }else{
                        tests_failed++;
                        /*Kernel::Plane_3 p1(vertices[i],vertices[(i+1)%3],vertices[(i+2)%3]);
                        Kernel::Plane_3 p2(vertices[i],vertices[(i+1)%3],*res);
                        std::cout << "Angle:     " << acos((p1.orthogonal_vector()*p2.orthogonal_vector())
                            /(sqrt(p1.orthogonal_vector().squared_length())*sqrt(p2.orthogonal_vector().squared_length())));*/
                        matplotArray scatterPoints;
                        matplotArray convexHull;

                        //input.emplace_back(input.fixPointSet.first);
                        //input.emplace_back(input.fixPointSet.second);

                        scatterPoints.addList(input);
                        CGAL::Polyhedron_3<Kernel> poly;
                        CGAL::convex_hull_3(input.begin(), input.end(), poly);

                        for(auto &p: poly.points()){
                            convexHull.addPoint(p);
                        }

                        matplotArray tri;
                        matplotArray triFalse;
                        tri.addPoint(vertices[0]);
                        tri.addPoint(vertices[1]);
                        tri.addPoint(vertices[2]);
                        tri.addPoint(vertices[0]);

                        triFalse.addPoint(vertices[i]);
                        triFalse.addPoint(*res);
                        triFalse.addPoint(vertices[(i + 1) % 3]);
                        triFalse.addPoint(vertices[i]);

                        plt::figure(1);
                        plt::clf();

                        plt::plot3(tri.x, tri.y, tri.z, {{"linewidth",  "0.2"},
                                                         {"marker",     "o"},
                                                         {"markersize", "0.0"},
                                                         {"color",      "g"}}, 1);

                        plt::plot3(triFalse.x, triFalse.y, triFalse.z, {{"linewidth",  "0.2"},
                                                         {"marker",     "o"},
                                                         {"markersize", "0.0"},
                                                         {"color",      "r"}}, 1);

                        /*plt::plot3(scatterPoints.x, scatterPoints.y, scatterPoints.z, {{"linewidth",  "0.0"},
                                                                                       {"marker",     "x"},
                                                                                       {"markersize", "2.5"}}, 1);
*/
                        plt::plot3(convexHull.x, convexHull.y, convexHull.z, {{"linewidth",  "0.0"},
                                                                              {"marker",     "x"},
                                                                              {"markersize", "2.5"},
                                                                              {"color",      "r"}}, 1);
                        plt::show();


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

TEST(FindBoundaryKD, C10M){
    /*for(int i = 0; i<100000; i++){
        testFindBoundaryKD(i,10);
    }*/
    testFindBoundaryKD(790,10);
}

TEST(CompareSearches, N100000){
    auto foo = [](const std::vector<Kernel::Point_3> &input){
        Kd_tree kd_tree(input.begin(), input.end(), Kd_tree::Splitter(1));
        kd_tree.build();

        Kernel::Point_3 origin(0, 0, 0);
        for (auto &p: kd_tree) {
            origin = {origin.x() + p.x() / kd_tree.size(), origin.y() + p.y() / kd_tree.size(),
                      origin.z() + p.z() / kd_tree.size()};
        }


        const CGAL::Kd_tree_rectangle<double, Traits::Dimension> &bbox(kd_tree.bounding_box());
        std::pair<Kernel::Point_3, Kernel::Point_3> set = {
                {bbox.max_coord(0), bbox.max_coord(1), bbox.max_coord(2)},
                {bbox.max_coord(0), bbox.max_coord(1), bbox.min_coord(2)}};

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        auto a = *findBoundaryPoint(kd_tree, set, BS_RIGHT, origin);

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        std::cout << input.size() << ";"<< std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() << ";";


        Kernel::Plane_3 u(set.first, set.second, origin);

        Kernel::Vector_3 normal(set.first, set.second); //vector along rotation axis
        normal /= sqrt(normal.squared_length());
        auto angle = 0;
        auto res = 0;

        begin = std::chrono::steady_clock::now();

        for (int i = 0; i < input.size(); i++) {
            Kernel::Plane_3 v(set.first, set.second, input[i]);
            auto a2 = atan2(
                    (CGAL::cross_product(u.orthogonal_vector(), v.orthogonal_vector())) * normal,
                    u.orthogonal_vector() * v.orthogonal_vector());
            if (angle > a2) {
                angle = a2;
                res = i;
            }
        }

        end = std::chrono::steady_clock::now();

        std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() << std::endl;
    };

    //generate input
    for (int i = 100; i < 1000000; i=i*1.5) {
        foo(generateInputVec3(i, 12345));
    }

}

