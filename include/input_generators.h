//
// Created by fabia on 01.02.2023.
//

#ifndef EXAMPLE_INPUT_GENERATORS_H
#define EXAMPLE_INPUT_GENERATORS_H

#include <vector>
#include <ctime>
#include "../external/glm/glm.hpp"


enum fixPointLocation {FPL_CONVEXHULL, FPL_RANDOM, FPL_USUALLY_INSIDE, FPL_CENTER};

template<typename PointType>
struct Input{
    std::vector<PointType> pointCloud;
    PointType fixPoint{};
};

Input<glm::vec2> generateInputVec2(int sample_size, int seed = std::time(nullptr),fixPointLocation hint = FPL_RANDOM, size_t x_dim = 100, size_t y_dim = 100);


#endif //EXAMPLE_INPUT_GENERATORS_H
