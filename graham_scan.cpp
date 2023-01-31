#include <cstdio>
#include <stack>
#include <algorithm>
#include <ctime>
#include <vector>
#include <iostream>
#include <chrono>
#include "glm/glm.hpp"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/ch_graham_andrew.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;

//
//  main.cpp
//  ConvexHull
//
//  Created by plaza on 8/24/15.
//  Copyright (c) 2015 __ComputationalGeometry__. All rights reserved.
//
//  Summary: Implementation of Graham's scan algorithm to compute the
//           convex hull of a set of points in the xy-plane
//
//
//  Input : A set of points in the xy-plane
//  Output: Two arrays containing the upper convex hull and the lower
//          convex hull of the input.

using namespace std;


// Point in the xy-plane
struct PointType{
    double x;
    double y;
};


// Sort criterion: points are sorted with respect to their x-coordinate.
//                 If two points have the same x-coordinate then we compare
//                 their y-coordinates
bool sortPoints(const PointType &lhs, const PointType &rhs)
{
    return (lhs.x < rhs.x) || (lhs.x==rhs.x && lhs.y < rhs.y);
}


// Check if three points make a right turn using cross product
bool right_turn(const PointType &P1, const PointType &P2, const PointType &P3)
{
    return ((P3.x-P1.x)*(P2.y-P1.y) - (P3.y-P1.y)*(P2.x-P1.x)) > 0;
}


int main () {
    int n_points;
    vector<PointType> lowerCH;
    vector<PointType> upperCH;

    std::vector<PointType> pointList;
    int sample_size =100000000;
    int seed = 1234;//std::time(nullptr);
    std::srand(seed);
    //generate point cloud and fixpoint
    int rand =std::rand();
    for (int i = 0; i<sample_size; i++) {
        pointList.push_back({sin(std::rand()) * 50, cos(std::rand()) * 50});
    }
    PointType fixPoint = {sin(rand) * 100, cos(rand) * 100};
    pointList.push_back(fixPoint);

    std::vector<Point_2> points;
    for(auto e: pointList){
        points.emplace_back(e.x,e.y);
    }
    std::vector<Point_2>  out;


    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    CGAL::ch_graham_andrew( points.begin(), points.end(), std::back_inserter(out) );

    for(auto &p: out){
        if(p.x() == fixPoint.x and p.y() == fixPoint.y){
            std::cout << fixPoint.x << "|" << fixPoint.y << "\n";
            break;
        }
    }



    end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[mircos]" << std::endl;
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;


    return 0;
}
/*int main()  {
    Point before=hull.top();
    Point current;
    Point after;
    while (!hull.empty())   {
        current = hull.top();
        if(current.x == fixPoint.x and current.y == fixPoint.y) break;
        hull.pop();
        before = current;
    }
    if(!hull.empty()) {
        hull.pop();
        after = hull.top();
    }


    std::cout <<before.x << "|" << before.y << "\n";
    std::cout <<current.x << "|" << current.y << "\n";
    std::cout <<after.x << "|" << after.y << "\n";
    std::cout << "\n";
    std::cout <<fixPoint.x << "|" << fixPoint.y << "\n";
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[mircos]" << std::endl;
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;
    return 0;
}
*/