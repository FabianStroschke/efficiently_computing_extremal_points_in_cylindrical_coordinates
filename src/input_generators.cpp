//
// Created by fabia on 01.02.2023.
//

#include "input_generators.h"

InputVec2
generateInputVec2(int sample_size, int seed, fixPointLocation hint, size_t x_dim, size_t y_dim) {
    InputVec2 res;

    //generate point cloud and fixpoint
    srand(seed);
    int r = std::rand();
    double scale = 1;

    switch (hint) {
        case FPL_CONVEXHULL:
            res.fixPoint = Kernel::Point_2(sin(r) * x_dim, cos(r) * y_dim);
            scale = 0.7;
            break;
        case FPL_RANDOM:
            res.fixPoint = Kernel::Point_2((float) (rand() % (x_dim * 2)) - x_dim, (float) (rand() % (y_dim * 2)) - y_dim);
            break;
        case FPL_USUALLY_INSIDE:
            res.fixPoint = Kernel::Point_2(sin(r) * x_dim / 4, cos(r) * y_dim / 4);
            break;
        case FPL_CENTER:
            res.fixPoint = Kernel::Point_2(0, 0);
            break;
    }

    for (int i = 0; i < sample_size; i++) {
        r = rand();
        res.pointCloud.emplace_back(sin(r) * (rand() % x_dim) * scale, cos(r) * (rand() % y_dim) * scale);
    }
    return res;
}

InputVec3
generateInputVec3(int sample_size, int seed, fixPointLocation hint, size_t x_dim, size_t y_dim, size_t z_dim) {
    InputVec3 res;

    //generate point cloud and fixpoint
    srand(seed);
    int r = std::rand();
    int r2 = std::rand();
    double scale = 1;

    switch (hint) {
        case FPL_CONVEXHULL: {
            res.fixPointSet.first = Kernel::Point_3(sin(r) * sin(r2) * x_dim, cos(r) * sin(r2) * y_dim,
                                                    cos(r2) * z_dim);
            auto v = CGAL::cross_product(
                    Kernel::Vector_3(res.fixPointSet.first, {0, 0, 0}),
                    Kernel::Vector_3(std::rand(), std::rand(), std::rand()));
            res.fixPointSet.second =
                    res.fixPointSet.first + (v / sqrt(v.squared_length())) * std::pow(x_dim * y_dim * z_dim, 0.3);
            scale = 0.7;
            break;
        }
        case FPL_RANDOM: {
            res.fixPointSet.first = Kernel::Point_3(
                    (float) (rand() % (x_dim * 2)) - x_dim,
                    (float) (rand() % (y_dim * 2)) - y_dim,
                    (float) (rand() % (z_dim * 2)) - z_dim);
            auto v = CGAL::cross_product(
                    Kernel::Vector_3(res.fixPointSet.first, {0, 0, 0}),
                    Kernel::Vector_3(std::rand(), std::rand(), std::rand()));
            res.fixPointSet.second =
                    res.fixPointSet.first + (v / sqrt(v.squared_length())) * std::pow(x_dim * y_dim * z_dim, 0.3);
            break;
        }
        case FPL_USUALLY_INSIDE:
            res.fixPointSet.first = Kernel::Point_3(
                    (float) (rand() % (x_dim * 2)) - x_dim,
                    (float) (rand() % (y_dim * 2)) - y_dim,
                    (float) (rand() % (z_dim * 2)) - z_dim);
            res.fixPointSet.second = Kernel::Point_3(sin(r) * sin(r2) * x_dim / 2, cos(r) * sin(r2) * y_dim / 2,
                                                     cos(r2) * z_dim / 2);
            break;
        case FPL_CENTER:
            res.fixPointSet.first = Kernel::Point_3(
                    (float) (rand() % (x_dim * 2)) - x_dim,
                    (float) (rand() % (y_dim * 2)) - y_dim,
                    (float) (rand() % (z_dim * 2)) - z_dim);
            res.fixPointSet.second = Kernel::Point_3(0, 0, 0);
            break;
    }

    for (int i = 0; i < sample_size; i++) {
        r = rand();
        r2 = rand();
        res.pointCloud.emplace_back(
                sin(r) * sin(r2) * (rand() % x_dim) * scale,
                cos(r) * sin(r2) * (rand() % y_dim) * scale,
                cos(r2) * (rand() % z_dim) * scale);
    }
    return res;
}

InputVec2 readInputVec2(std::string path) {
    InputVec2 res;

    std::vector<Kernel::Point_3> points;
    std::vector<std::vector<std::size_t> > polygons;

    CGAL::IO::read_polygon_soup(path, points, polygons);

    for(auto &p:points){
        res.pointCloud.emplace_back(p.x(), p.y());
    }
    res.fixPoint = {0,0};

    return res;
}

InputVec3 readInputVec3(std::string path) {
    InputVec3 res;

    std::vector<std::vector<std::size_t> > polygons;

    CGAL::IO::read_polygon_soup(path, res.pointCloud, polygons);

    res.fixPointSet = {{0,0,0},{0,0,0}};

    return res;
}

