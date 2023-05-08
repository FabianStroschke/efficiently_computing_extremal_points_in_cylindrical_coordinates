//
// Created by fabia on 01.02.2023.
//

#ifndef EXAMPLE_INPUT_GENERATORS_H
#define EXAMPLE_INPUT_GENERATORS_H

#include <vector>
#include <ctime>
#include "../external/glm/glm.hpp"
#include "CGALSetup.h"
#include <CGAL/IO/polygon_soup_io.h>


enum fixPointLocation {
    FPL_CONVEXHULL, FPL_RANDOM, FPL_USUALLY_INSIDE, FPL_CENTER
};


struct InputVec2 {
    std::vector<Kernel::Point_2> pointCloud;
    Kernel::Point_2 fixPoint{};
};

struct InputVec3 {
    std::vector<Kernel::Point_3> pointCloud;
    std::pair<Kernel::Point_3, Kernel::Point_3> fixPointSet;
};


InputVec2 generateInputVec2(int sample_size, int seed = std::time(nullptr), fixPointLocation hint = FPL_RANDOM,
                  size_t x_dim = 100, size_t y_dim = 100);

InputVec3 generateInputVec3(int sample_size, int seed = std::time(nullptr), fixPointLocation hint = FPL_RANDOM,
                            size_t x_dim = 100, size_t y_dim = 100, size_t z_dim = 100);

InputVec2 readInputVec2(std::string path);
InputVec3 readInputVec3(std::string path);

#endif //EXAMPLE_INPUT_GENERATORS_H
