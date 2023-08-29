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
    auto input = generateInputVec3(sample_size, seed);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    Mesh res;
    KDWrap(input, res);

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

        //input.emplace_back(input.fixPointSet.first);
        //input.emplace_back(input.fixPointSet.second);

        scatterPoints.addList(input);
        CGAL::Polyhedron_3<Kernel> poly;
        CGAL::convex_hull_3(input.begin(), input.end(), poly);

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
        /*for (auto e = res.halfedges_begin(); e != res.halfedges_end(); e++) {
            matplotArray edge;
            edge.addPoint(res.points()[res.target(*e)]);
            edge.addPoint(res.points()[res.source(*e)]);
            edges.push_back(edge);
            plt::plot3(edges.back().x, edges.back().y, edges.back().z, {{"linewidth", "0.5"},
                                                                        {"color",     "c"}}, 1);
        }*/

        Kd_tree kd_tree(input.begin(),input.end(),Kd_tree::Splitter(1));
        kd_tree.build();
        matplotKDtree treeBoxes(kd_tree);
        treeBoxes.show();


        Kernel::Point_3 origin(0,0,0);
        for(auto &p: kd_tree){
            origin = {origin.x()+p.x()/kd_tree.size(),origin.y()+p.y()/kd_tree.size(),origin.z()+p.z()/kd_tree.size()};
        }


        problemEdges.addPoint({-36.258379764854524, -4.1599316927573398, 3.0090613243501325});
        problemEdges.addPoint({-0.53957021850918252, -3.8692498521924357, 65.679343979892749});
        problemEdges.addPoint({-12.812, -7.90927, 32.1854});

        problemPoints.addPoint({10.7977, -34.6158, -10.2332});
        problemPoints.addPoint(origin);


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