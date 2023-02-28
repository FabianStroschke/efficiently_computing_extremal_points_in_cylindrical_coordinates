//
// Created by fabia on 01.02.2023.
//

#ifndef EXAMPLE_INPUT_GENERATORS_H
#define EXAMPLE_INPUT_GENERATORS_H

#include <vector>
#include <ctime>
#include "../external/glm/glm.hpp"
#include "CGALSetup.h"



enum fixPointLocation {FPL_CONVEXHULL, FPL_RANDOM, FPL_USUALLY_INSIDE, FPL_CENTER};

template<typename PointType>
struct InputVec2{
    std::vector<PointType> pointCloud;
    PointType fixPoint{};
};
template<typename PointType>
InputVec2<PointType> generateInputVec2(int sample_size, int seed = std::time(nullptr), fixPointLocation hint = FPL_RANDOM, size_t x_dim = 100, size_t y_dim = 100);


struct InputVec3{
    std::vector<Kernel::Point_3> pointCloud;
    std::pair<Kernel::Point_3,Kernel::Point_3> fixPointSet;
};
InputVec3 generateInputVec3(int sample_size, int seed = std::time(nullptr), fixPointLocation hint = FPL_RANDOM, size_t x_dim = 100, size_t y_dim = 100, size_t z_dim = 100);

#endif //EXAMPLE_INPUT_GENERATORS_H
