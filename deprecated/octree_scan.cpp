//
// Created by fabia on 06.03.2023.
//
#include "include/input_generators.h"
#include "include/config.h"
#include "include/tree_scan_helper.h"

#include "CGALSetup.h"
#include <CGAL/Octree.h>


//typedefs
typedef Kernel::Point_3 Point_3;
typedef CGAL::Octree<Kernel, std::vector<Kernel::Point_3>> Octree;

std::vector<Kernel::Point_3>
octreeScan(std::vector<Kernel::Point_3> &pointCloud, std::pair<Kernel::Point_3,Kernel::Point_3> &fixPointSet);

int main() {
    auto input = generateInputVec3(sample_size, seed, FPL_CONVEXHULL);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    auto res = octreeScan(input.pointCloud, input.fixPointSet);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Time difference = "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]"
              << std::endl;
    std::cout << "Time difference = "
              << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[mircos]"
              << std::endl;
    std::cout << "Time difference = "
              << std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() << "[ns]"
              << std::endl;

    //std::cout << counter << std::endl;
    std::cout << "seed:" << seed << std::endl;

}

std::vector<Kernel::Point_3>
octreeScan(std::vector<Kernel::Point_3> &pointCloud, std::pair<Kernel::Point_3,Kernel::Point_3> &fixPointSet){
    std::vector<Kernel::Point_3> res;
    /** Building the Octree **/

    Octree octree(pointCloud);
    octree.refine(10, 15);

    /** Solving the Problem here **/

    Kernel::Point_3 const *res1 = findBoundaryPoint(octree, fixPointSet, BS_LEFT);
    if(res1 != nullptr){
        res.emplace_back(*res1);
    }
    Kernel::Point_3 const *res2 = findBoundaryPoint(octree, fixPointSet, BS_RIGHT);
    if(res2 != nullptr){
        res.emplace_back(*res2);
    }
    return res;
}

