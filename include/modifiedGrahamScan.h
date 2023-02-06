//
// Created by fabia on 01.02.2023.
//

#ifndef EXAMPLE_MODIFIEDGRAHAMSCAN_H
#define EXAMPLE_MODIFIEDGRAHAMSCAN_H

#include <vector>
#include "../external/glm/glm.hpp"
#include "matplotlibcpp.h"

std::vector<glm::vec2> modifiedGrahamScanVec2(std::vector<glm::vec2> &pointCloud, glm::vec2 &fixPoint, bool show = false);

#endif //EXAMPLE_MODIFIEDGRAHAMSCAN_H
