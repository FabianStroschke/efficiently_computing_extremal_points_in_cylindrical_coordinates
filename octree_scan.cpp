//
// Created by fabia on 06.03.2023.
//
#include "include/input_generators.h"
#include "include/config.h"

#include "CGALSetup.h"
#include <CGAL/Octree.h>

#include "external/matplotlibcpp/matplotlibcpp.h"

//namespaces
namespace plt = matplotlibcpp;

//typedefs
typedef Kernel::Point_3 Point_3;
typedef CGAL::Octree<Kernel, std::vector<Kernel::Point_3>> Octree;

int main() {
    auto input = generateInputVec3(sample_size, seed, FPL_RANDOM);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    /** Building the Octree **/

    Octree octree(input.pointCloud);
    octree.refine(10, 15);

    std::chrono::steady_clock::time_point ocTreeFinish = std::chrono::steady_clock::now();

    /** Solving the Problem here **/
    //for index order see: https://doc.cgal.org/latest/Orthtree/classCGAL_1_1Orthtree_1_1Node.html#a706069ea795fdf65b289f597ce1eb8fd

    Kernel::Point_2 const *res = findBoundaryPoint(quadtree, input.fixPoint, BS_LEFT);
    Kernel::Point_2 const *res2 = findBoundaryPoint(quadtree, input.fixPoint, BS_RIGHT);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Time difference = "
              << std::chrono::duration_cast<std::chrono::milliseconds>(ocTreeFinish - begin).count() << "[ms]"
              << std::endl;
    std::cout << "Time difference = "
              << std::chrono::duration_cast<std::chrono::microseconds>(ocTreeFinish - begin).count() << "[mircos]"
              << std::endl;
    std::cout << "Time difference = "
              << std::chrono::duration_cast<std::chrono::nanoseconds>(ocTreeFinish - begin).count() << "[ns]"
              << std::endl;

    std::cout << "Time difference = "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - ocTreeFinish).count() << "[ms]"
              << std::endl;
    std::cout << "Time difference = "
              << std::chrono::duration_cast<std::chrono::microseconds>(end - ocTreeFinish).count() << "[mircos]"
              << std::endl;
    std::cout << "Time difference = "
              << std::chrono::duration_cast<std::chrono::nanoseconds>(end - ocTreeFinish).count() << "[ns]"
              << std::endl;

    //std::cout << counter << std::endl;
    std::cout << "seed:" << seed << std::endl;
}