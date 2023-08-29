//
// Created by fabia on 01.02.2023.
//

#include "input_generators.h"

std::vector<Kernel::Point_3>
generateInputVec3(int sample_size, int seed,  size_t x_dim, size_t y_dim, size_t z_dim) {
    std::vector<Kernel::Point_3> res;

    //generate point cloud and fixpoint
    if(seed != -1) srand(seed);
    int r = std::rand();
    int r2 = std::rand();
    double scale = 1;

    for (int i = 0; i < sample_size; i++) {
        r = rand();
        r2 = rand();
        res.emplace_back(
                sin(r) * sin(r2) * (rand() % x_dim) * scale,
                cos(r) * sin(r2) * (rand() % y_dim) * scale,
                cos(r2) * (rand() % z_dim) * scale);
    }
    return res;
}

std::vector<Kernel::Point_3> readInputVec3(std::string path) {
    std::vector<Kernel::Point_3> res;

    std::vector<std::vector<std::size_t> > polygons;

    CGAL::IO::read_polygon_soup(path, res, polygons);

    return res;
}
