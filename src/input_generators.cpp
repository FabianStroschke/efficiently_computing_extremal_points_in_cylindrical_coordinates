//
// Created by fabia on 01.02.2023.
//

#include "input_generators.h"

std::vector<Kernel::Point_3>
generateInputVec3(int sample_size, int seed,  double x_dim, double y_dim, double z_dim) {
    std::vector<Kernel::Point_3> res;
    if(seed != -1) srand(seed);
    int r;
    int r2;

    for (int i = 0; i < sample_size; i++) {
        r = rand();
        r2 = rand();

        res.emplace_back(
                sin(r) * sin(r2) * (((double)rand() / (double)RAND_MAX)*x_dim/2),
                cos(r) * sin(r2) * (((double)rand() / (double)RAND_MAX)*y_dim/2),
                cos(r2)             * (((double)rand() / (double)RAND_MAX)*z_dim/2));
    }
    return res;
}

std::vector<Kernel::Point_3> readInputVec3(std::string path) {
    std::vector<Kernel::Point_3> res;

    std::vector<std::vector<std::size_t> > polygons;

    CGAL::IO::read_polygon_soup(path, res, polygons);

    return res;
}
