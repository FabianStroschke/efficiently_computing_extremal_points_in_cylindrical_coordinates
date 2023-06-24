//
// Created by fabia on 06.03.2023.
//
#include "input_generators.h"
#include "config.h"
#include "octreeWrap.h"


#include "external/matplotlibcpp/matplotlibcpp.h"
#include "matplot_helper.h"

//namespaces
namespace plt = matplotlibcpp;

int main() {
    auto input = generateInputVec3(sample_size, seed, FPL_CONVEXHULL);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    Mesh res;
    octreeWrap(input.pointCloud, res);

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

    if (MatPlotShow) {
        //Matplot
        matplotArray scatterPoints;
        matplotArray convexHull;
        matplotArray start;

        //input.pointCloud.emplace_back(input.fixPointSet.first);
        //input.pointCloud.emplace_back(input.fixPointSet.second);

        scatterPoints.addList(input.pointCloud);
        for (auto &p: res.points()) {
            convexHull.addPoint(p);
        }
        start.addPoint(res.points().begin()[0]);
        start.addPoint(res.points().begin()[1]);
        start.addPoint(res.points().begin()[2]);

        plt::figure(1);
        plt::clf();
        plt::plot3(scatterPoints.x, scatterPoints.y, scatterPoints.z, {{"linewidth",  "0.0"},
                                                                          {"marker",     "x"},
                                                                          {"markersize", "2.5"}}, 1);

        plt::plot3(convexHull.x, convexHull.y, convexHull.z, {{"linewidth",  "0.0"},
                                                                 {"marker",     "x"},
                                                                 {"markersize", "2.5"},
                                                                 {"color",      "r"}}, 1);
        plt::plot3(start.x, start.y, start.z, {{"linewidth",  "0.0"},
                                                  {"marker",     "o"},
                                                  {"markersize", "5.0"},
                                                  {"color",      "g"}}, 1);


        std::vector<matplotArray> edges;


        for (auto e = res.halfedges_begin(); e != res.halfedges_end(); e++) {
            matplotArray edge;
            edge.addPoint(res.points()[res.target(*e)]);
            edge.addPoint(res.points()[res.source(*e)]);

            edges.push_back(edge);
            plt::plot3(edges.back().x, edges.back().y, edges.back().z, {{"linewidth", "0.5"},
                                                                           {"color",     "c"}}, 1);


        }

        plt::show();
    }


}


