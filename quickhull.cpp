#include "include/input_generators.h"
#include "include/config.h"

#include "CGALSetup.h"
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>
#include "external/matplotlibcpp/matplotlibcpp.h"

//namespaces
namespace plt = matplotlibcpp;

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

    if (MatPlotShow) {
        //Matplot
        std::vector<std::vector<double>> scatterPoints(3);
        std::vector<std::vector<double>> fixPointSet(3);
        std::vector<std::vector<double>> convexHull(3);

        //input.emplace_back(input.fixPointSet.first);
        //input.emplace_back(input.fixPointSet.second);

        for (auto &p: input) {
            scatterPoints[0].emplace_back(p.x());
            scatterPoints[1].emplace_back(p.y());
            scatterPoints[2].emplace_back(p.z());
        }


        for (auto &p: poly.points()) {
            convexHull[0].emplace_back(p.x());
            convexHull[1].emplace_back(p.y());
            convexHull[2].emplace_back(p.z());
        }

        plt::figure(1);
        plt::clf();
        plt::plot3(scatterPoints[0], scatterPoints[1], scatterPoints[2], {{"linewidth",  "0.0"},
                                                                          {"marker",     "x"},
                                                                          {"markersize", "2.5"}}, 1);

        plt::plot3(convexHull[0], convexHull[1], convexHull[2], {{"linewidth",  "0.0"},
                                                                 {"marker",     "x"},
                                                                 {"markersize", "2.5"},
                                                                 {"color",      "r"}}, 1);

        std::vector<std::vector<std::vector<double>>> edges;


        for (auto e = poly.halfedges_begin(); e != poly.halfedges_end(); e++) {
            std::vector<std::vector<double>> edge(3);
            auto &p1 = e->vertex()->point();
            auto &p2 = e->next()->vertex()->point();

            edge[0].emplace_back(p1.x());
            edge[0].emplace_back(p2.x());
            edge[1].emplace_back(p1.y());
            edge[1].emplace_back(p2.y());
            edge[2].emplace_back(p1.z());
            edge[2].emplace_back(p2.z());

            edges.push_back(edge);
            plt::plot3(edges.back()[0], edges.back()[1], edges.back()[2], {{"linewidth", "0.5"},
                                                                           {"color",     "c"}}, 1);


        }

        plt::show();
    }

}