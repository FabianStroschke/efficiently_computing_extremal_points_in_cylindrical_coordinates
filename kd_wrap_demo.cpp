//
// Created by fabia on 14.07.2023.
//

#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>
#include "kd_wrap.h"
#include "input_generators.h"
#include "config.h"
#include "matplot_helper.h"

int main() {
    auto input = generateInputVec3(50,66, FPL_CONVEXHULL);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    Mesh res;
    KDWrap(input.pointCloud, res);

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
        matplotArray problemPoints;
        matplotArray problemEdges;

        //input.pointCloud.emplace_back(input.fixPointSet.first);
        //input.pointCloud.emplace_back(input.fixPointSet.second);

        scatterPoints.addList(input.pointCloud);
        CGAL::Polyhedron_3<Kernel> poly;
        CGAL::convex_hull_3(input.pointCloud.begin(), input.pointCloud.end(), poly);

        for(auto &p: poly.points()){
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


        for (auto e = poly.halfedges_begin(); e != poly.halfedges_end(); e++) {
            matplotArray edge;
            edge.addPoint(e->vertex()->point());
            edge.addPoint(e->prev()->vertex()->point());

            edges.push_back(edge);
            plt::plot3(edges.back().x, edges.back().y, edges.back().z, {{"linewidth", "0.5"},
                                                                        {"color",     "c"}}, 1);


        }

        Kd_tree kd_tree(input.pointCloud.begin(),input.pointCloud.end(),Kd_tree::Splitter(1));
        kd_tree.build();
        matplotKDtree treeBoxes(kd_tree);
        //treeBoxes.show();

        problemEdges.addPoint({-7.8092752464774433, 31.225320294365883, 19.627871754554246});
        problemEdges.addPoint({35.072180730027412, -5.6753827417977991, -55.331917295176702});
        problemEdges.addPoint({-14.8482, 51.9199, -2.55789});


        problemEdges.addPoint({35.072180730027412, -5.6753827417977991, -55.331917295176702});
        problemEdges.addPoint({-7.8092752464774433, 31.225320294365883, 19.627871754554246});
        problemEdges.addPoint({3.93538, 0.0425144, 37.6778});

        problemPoints.addPoint({0.589499, -3.77091, -38.5859});
        problemPoints.addPoint({0.0494749, -0.0893824, -53.1999});
        problemPoints.addPoint({(kd_tree.bounding_box().max_coord(0) + kd_tree.bounding_box().min_coord(0)) / 2, (kd_tree.bounding_box().max_coord(1) + kd_tree.bounding_box().min_coord(1)) / 2, (kd_tree.bounding_box().max_coord(2) + kd_tree.bounding_box().min_coord(2)) / 2});


        plt::plot3(problemEdges.x, problemEdges.y, problemEdges.z, {{"linewidth",  "0.0"},
                                                                    {"marker",     "o"},
                                                                    {"markersize", "5.0"},
                                                                    {"color",      "b"}}, 1);


        plt::plot3(problemPoints.x, problemPoints.y, problemPoints.z, {{"linewidth",  "0.0"},
                                                                    {"marker",     "o"},
                                                                    {"markersize", "5.0"},
                                                                    {"color",      "r"}}, 1);




        plt::show();
    }
}