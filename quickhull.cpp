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

std::vector<Kernel::Point_3>
quickhullScan(std::vector<Kernel::Point_3> &pointCloud, std::pair<Kernel::Point_3,Kernel::Point_3> &fixPointSet);

int main() {
    auto input = readInputVec3("../inputs/suzanne.obj");//generateInputVec3(sample_size, seed, FPL_CONVEXHULL);
    randomizeFixpointVec3(input, FPL_RANDOM, seed);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    auto res = quickhullScan(input.pointCloud, input.fixPointSet);

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

        input.pointCloud.emplace_back(input.fixPointSet.first);
        input.pointCloud.emplace_back(input.fixPointSet.second);

        for (auto &p: input.pointCloud) {
            scatterPoints[0].emplace_back(p.x());
            scatterPoints[1].emplace_back(p.y());
            scatterPoints[2].emplace_back(p.z());
        }
        Polyhedron_3 poly;
        CGAL::convex_hull_3(input.pointCloud.begin(), input.pointCloud.end(), poly);

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

        fixPointSet[0].emplace_back(input.fixPointSet.first.x());
        fixPointSet[0].emplace_back(input.fixPointSet.second.x());
        fixPointSet[1].emplace_back(input.fixPointSet.first.y());
        fixPointSet[1].emplace_back(input.fixPointSet.second.y());
        fixPointSet[2].emplace_back(input.fixPointSet.first.z());
        fixPointSet[2].emplace_back(input.fixPointSet.second.z());

        if (not res.empty()) {
            fixPointSet[0].emplace_back(res[0].x());
            fixPointSet[0].emplace_back(input.fixPointSet.first.x());
            fixPointSet[0].emplace_back(res[1].x());
            fixPointSet[0].emplace_back(input.fixPointSet.second.x());

            fixPointSet[1].emplace_back(res[0].y());
            fixPointSet[1].emplace_back(input.fixPointSet.first.y());
            fixPointSet[1].emplace_back(res[1].y());
            fixPointSet[1].emplace_back(input.fixPointSet.second.y());

            fixPointSet[2].emplace_back(res[0].z());
            fixPointSet[2].emplace_back(input.fixPointSet.first.z());
            fixPointSet[2].emplace_back(res[1].z());
            fixPointSet[2].emplace_back(input.fixPointSet.second.z());
            plt::plot3(fixPointSet[0], fixPointSet[1], fixPointSet[2], {{"linewidth", "1.0"},
                                                                        {"color",     "g"}}, 1);
        } else {
            plt::plot3(fixPointSet[0], fixPointSet[1], fixPointSet[2], {{"linewidth", "1.0"},
                                                                        {"color",     "r"}}, 1);
        }


        plt::show();
    }

}

std::vector<Kernel::Point_3>
quickhullScan(std::vector<Kernel::Point_3> &pointCloud, std::pair<Kernel::Point_3,Kernel::Point_3> &fixPointSet){
    std::vector<Kernel::Point_3> res;

    pointCloud.emplace_back(fixPointSet.first);
    pointCloud.emplace_back(fixPointSet.second);

    Polyhedron_3 poly;
    // compute convex hull of non-collinear points
    CGAL::convex_hull_3(pointCloud.begin(), pointCloud.end(), poly);


    bool resExists = false;
    for (auto e = poly.halfedges_begin(); e != poly.halfedges_end(); e++) {
        if (e->vertex()->point() == fixPointSet.first
            and e->next()->vertex()->point() == fixPointSet.second) {
            resExists = true;

            //find first solution
            auto f = e->facet_begin();
            while (f->vertex()->point() == fixPointSet.first or
                   f->vertex()->point() == fixPointSet.second)
                f++;
            res.emplace_back(f->vertex()->point());

            //find second solution
            f = e->next()->opposite()->facet_begin();
            while (f->vertex()->point() == fixPointSet.first or
                   f->vertex()->point() == fixPointSet.second)
                f++;
            res.emplace_back(f->vertex()->point());

            break;
        }
    }

    //remove fixpoint from pointCloud to restore its original state
    pointCloud.pop_back();
    pointCloud.pop_back();
    return res;
}