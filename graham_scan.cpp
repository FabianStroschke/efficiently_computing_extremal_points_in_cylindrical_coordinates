#include <stack>
#include <chrono>
#include <vector>
#include <iostream>
#include "include/input_generators.h"
#include "include/config.h"

#include "CGALSetup.h"
#include <CGAL/ch_graham_andrew.h>

std::vector<Kernel::Point_2>
grahamScanVec2(std::vector<Kernel::Point_2> &pointCloud, Kernel::Point_2 &fixPoint);

int main() {
    auto input = generateInputVec2<Kernel::Point_2>(sample_size, seed, FPL_CONVEXHULL);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    auto res = grahamScanVec2(input.pointCloud, input.fixPoint);

    end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()
              << "[ms]" << std::endl;
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()
              << "[mircos]" << std::endl;
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count()
              << "[ns]" << std::endl;


    return 0;
}

std::vector<Kernel::Point_2>
grahamScanVec2(std::vector<Kernel::Point_2> &pointCloud, Kernel::Point_2 &fixPoint){
    std::vector<Kernel::Point_2> res(2);
    std::vector<Kernel::Point_2> out;

    pointCloud.push_back(
            fixPoint); //add fixpoint to point cloud, so that graham scan can find a convex hull containing the fixpoint

    CGAL::ch_graham_andrew(pointCloud.begin(), pointCloud.end(), std::back_inserter(out));

    for (int i = 0; i < out.size(); i++) {
        if (out[i].x() == fixPoint.x() and out[i].y() == fixPoint.y()) {
            res[0] = out[(i + out.size() - 1) % out.size()];
            res[1] = out[(i + out.size() + 1) % out.size()];
            break;
        }
    }
    pointCloud.pop_back(); //remove fixpoint from pointCloud to restore its original state
    return res;
}