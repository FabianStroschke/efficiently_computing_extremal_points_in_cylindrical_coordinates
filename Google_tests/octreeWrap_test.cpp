//
// Created by fabia on 22.06.2023.
//
#include "gtest/gtest.h"
#include "octreeWrap.h"
#include "input_generators.h"

#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>

//typedefs
typedef CGAL::Polyhedron_3<Kernel> Polyhedron_3;

void testSameConvexHullRandom(long seed, long nPoints){

    //generate input
    auto input = generateInputVec3(nPoints, seed, FPL_CONVEXHULL);

    //build convex hull
    Polyhedron_3 poly;
    CGAL::convex_hull_3(input.pointCloud.begin(), input.pointCloud.end(), poly);

    //build convex mesh
    Mesh mesh;
    octreeWrap(input.pointCloud, mesh);

    for(auto &p: poly.points()){
        EXPECT_NE(std::find(mesh.points().begin(), mesh.points().end(),p),mesh.points().end());
    }
}

void testSameConvexHullInput(std::string path){
    //generate input
    auto input = readInputVec3(path);

    //build convex hull
    Polyhedron_3 poly;
    CGAL::convex_hull_3(input.pointCloud.begin(), input.pointCloud.end(), poly);

    //build convex mesh
    Mesh mesh;
    octreeWrap(input.pointCloud, mesh);

    for(auto &p: poly.points()){
        EXPECT_NE(std::find(mesh.points().begin(), mesh.points().end(),p),mesh.points().end())<<"Failed with object: " << path;
    }
}

TEST(CheckConvexHull, N500){
    int n = 500;
    testSameConvexHullRandom(1, n);
    testSameConvexHullRandom(13, n);
    testSameConvexHullRandom(345, n);
    testSameConvexHullRandom(7345, n);
    testSameConvexHullRandom(98175, n);
    testSameConvexHullRandom(123456, n);
    testSameConvexHullRandom(9807, n);
    testSameConvexHullRandom(468014, n);
}

TEST(CheckConvexHull, N5000){
    int n = 5000;
    testSameConvexHullRandom(1, n);
    testSameConvexHullRandom(13, n);
    testSameConvexHullRandom(345, n);
    testSameConvexHullRandom(7345, n);
    testSameConvexHullRandom(98175, n);
    testSameConvexHullRandom(123456, n);
    testSameConvexHullRandom(9807, n);
    testSameConvexHullRandom(468014, n);
}

TEST(CheckConvexHull, N50000){
    int n = 50000;
    testSameConvexHullRandom(1, n);
    testSameConvexHullRandom(13, n);
    testSameConvexHullRandom(345, n);
    testSameConvexHullRandom(7345, n);
    testSameConvexHullRandom(98175, n);
    testSameConvexHullRandom(123456, n);
    testSameConvexHullRandom(9807, n);
    testSameConvexHullRandom(468014, n);
}

TEST(CheckConvexHull, Objects){
    testSameConvexHullInput("../inputs/alligator.obj");
    testSameConvexHullInput("../inputs/armadillo.obj");
    testSameConvexHullInput("../inputs/beast.obj");
    testSameConvexHullInput("../inputs/beetle.obj");
    testSameConvexHullInput("../inputs/beetle-alt.obj");
    testSameConvexHullInput("../inputs/bimba.obj");
    testSameConvexHullInput("../inputs/cow.obj");
    testSameConvexHullInput("../inputs/fandisk.obj");
    testSameConvexHullInput("../inputs/happy.obj");
    testSameConvexHullInput("../inputs/homer.obj");
    testSameConvexHullInput("../inputs/horse.obj");
    testSameConvexHullInput("../inputs/igea.obj");
    testSameConvexHullInput("../inputs/lucy.obj");
    testSameConvexHullInput("../inputs/orge.obj");
}
