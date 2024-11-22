#include "include/input_generators.h"
#include "include/config.h"

#include "CGALSetup.h"
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>

//typedefs
typedef CGAL::Polyhedron_3<Kernel> Polyhedron_3;

int main() {
    auto input = generateInputVec3(sample_size, seed);//readInputVec3("../inputs/suzanne.obj");

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    Polyhedron_3 poly;
    CGAL::convex_hull_3(input.begin(), input.end(), poly);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()
              << "[ms]" << std::endl;
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()
              << "[mircos]" << std::endl;
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count()
              << "[ns]" << std::endl;

}