//
// Created by fabia on 01.02.2023.
//

#include "input_generators.h"

template<typename PointType>
Input<PointType> generateInputVec2(int sample_size, int seed, fixPointLocation hint, size_t x_dim, size_t y_dim) {
    Input<PointType> res;

    //generate point cloud and fixpoint
    srand(seed);
    int r = std::rand();
    double scale = 1;

    switch (hint) {
        case FPL_CONVEXHULL:
            res.fixPoint = PointType(sin(r) * x_dim, cos(r) * y_dim);
            scale = 0.7;
            break;
        case FPL_RANDOM:
            res.fixPoint = PointType((float )(rand() % (x_dim * 2)) - x_dim, (float )(rand() % (y_dim * 2)) - y_dim);
            break;
        case FPL_USUALLY_INSIDE:
            res.fixPoint = PointType(sin(r) * x_dim/4, cos(r) * y_dim/4);
            break;
        case FPL_CENTER:
            res.fixPoint = PointType(0,0);
            break;
    }

    for (int i = 0; i<sample_size; i++) {
        r = rand();
        res.pointCloud.emplace_back(sin(r) * (rand()%x_dim)*scale, cos(r) * (rand()%y_dim)*scale);
    }
    return res;
}


template
Input<CGAL::Point_2<CGAL::Epick> > generateInputVec2<CGAL::Point_2<CGAL::Epick> >(int sample_size, int seed, fixPointLocation hint, size_t x_dim, size_t y_dim);

template
Input<glm::vec2> generateInputVec2<glm::vec2>(int sample_size, int seed, fixPointLocation hint, size_t x_dim, size_t y_dim);
