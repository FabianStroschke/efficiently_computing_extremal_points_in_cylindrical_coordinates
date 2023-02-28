#include "include/input_generators.h"
#include "CGALSetup.h"
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/convex_hull_3.h>
#include "matplotlibcpp.h"

//namespaces
namespace plt = matplotlibcpp;

//typedefs
typedef CGAL::Polyhedron_3<Kernel>                     Polyhedron_3;
typedef Kernel::Point_3                                Point_3;
typedef CGAL::Surface_mesh<Point_3>                    Surface_mesh;

int main () {
    int sample_size =50;
    int seed = std::time(nullptr);

    auto input = generateInputVec3(sample_size, seed, FPL_CONVEXHULL);


    bool show = true;
    if(show) {
        //Matplot
        std::vector<std::vector<double>> scatterPoints(3);
        std::vector<std::vector<double>> fixPointSet(3);

        for (auto &p: input.pointCloud) {
            scatterPoints[0].emplace_back(p.x());
            scatterPoints[1].emplace_back(p.y());
            scatterPoints[2].emplace_back(p.z());
        }
        fixPointSet[0].emplace_back(input.fixPointSet.first.x());
        fixPointSet[0].emplace_back(input.fixPointSet.second.x());
        fixPointSet[1].emplace_back(input.fixPointSet.first.y());
        fixPointSet[1].emplace_back(input.fixPointSet.second.y());
        fixPointSet[2].emplace_back(input.fixPointSet.first.z());
        fixPointSet[2].emplace_back(input.fixPointSet.second.z());

        plt::figure(1);
        plt::clf();
        plt::plot3(scatterPoints[0], scatterPoints[1], scatterPoints[2], {{"linewidth",  "0.0"},
                                                       {"marker",     "x"},
                                                       {"markersize", "2.5"}},1);
        plt::plot3(fixPointSet[0],fixPointSet[1],fixPointSet[2], {{"linewidth","1.0"}},1);

        plt::show();
    }

}