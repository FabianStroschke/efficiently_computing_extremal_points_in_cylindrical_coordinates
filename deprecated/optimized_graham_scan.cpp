#include <fstream>
#include "external/glm/glm.hpp"
#include "include/input_generators.h"
#include "include/config.h"
#include <vector>


#define log_function_time(f, oStream) std::chrono::steady_clock::time_point logger_begin = std::chrono::steady_clock::now(); f;  std::chrono::steady_clock::time_point logger_end = std::chrono::steady_clock::now(); log <<  std::chrono::duration_cast<std::chrono::microseconds>(logger_end - logger_begin).count();

std::vector<Kernel::Point_2>
optimizedGrahamScanVec2(std::vector<Kernel::Point_2> &pointCloud, Kernel::Point_2 &fixPoint, bool show = false);

int main() {
    srand(seed);

    std::ofstream log;
    log.open("log.txt", std::ios::out | std::ios::app);

    //seed = std::time(nullptr)+i;
    auto input = readInputVec2("../inputs/suzanne.obj");//generateInputVec2(sample_size, seed, FPL_CONVEXHULL);
    randomizeFixpointVec2(input, FPL_CONVEXHULL);
    log << sample_size << ",";
    log_function_time(auto res = optimizedGrahamScanVec2(input.pointCloud, input.fixPoint), log);
    log << "\n";
    std::cout << res[0].x() << "|" << res[0].y() << "\n";
    std::cout << res[1].x() << "|" << res[1].y() << "\n";
    std::cout << "_____________________________________________\n";
    std::cout << seed << "\n";

    log.close();
}

struct angleVec2Pair {
    double angle;
    Kernel::Point_2 vec;

    angleVec2Pair(double a, Kernel::Point_2 v) : angle(a), vec(v) {};

    bool operator<(const angleVec2Pair &other) const {
        return angle < other.angle;
    }
};

std::vector<Kernel::Point_2> optimizedGrahamScanVec2(std::vector<Kernel::Point_2> &pointCloud, Kernel::Point_2 &fixPoint, bool show) {
    std::vector<std::vector<angleVec2Pair>> vectorList(360);
    for (auto &p: pointCloud) {
        auto angle = atan2(fixPoint.y() - p.y(), fixPoint.x() - p.x()) * 180 / M_PI + 180; //TODO: vlt zu p-fixpoint ändern, damit die rotation klarer um den fixpunkt geht
        if (angle > 360)angle -= 360;
        vectorList[(int) angle].emplace_back(angle, p);
    }
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> x1;
    std::vector<float> y1;
    std::pair<std::vector<float>, std::vector<float>> line;

    int offset = -1;
    double min = 0;
    double max = 0;
    double angle = 0;
    for (int i = 0; i < vectorList.size(); i++) {

        if (not vectorList[i].empty()) {
            if (offset == -1) offset = i;
            if (angle > max - min) {
                min = i - angle;
                max = i;
            }
            angle = 0;
        }
        angle++;
    }
    if (angle + offset > max - min) {
        min = vectorList.size() - angle;
        max = offset;
    }
    /**min und max haben immer den richtigen bucket, falls eine lösung exisiert, weil die lösungs vektoren immer
     * mindestens 180° versetzt sein müssen und dann nur ein paar exisieren kann
     * SONDERFALL TODO: sind die punkte nur in 2 buckets die genau 180° versetzt sind kann das ergebins falsch sein
    **/
    angleVec2Pair vmin = *(std::max_element(vectorList[min].begin(), vectorList[min].end()));
    angleVec2Pair vmax = *(std::min_element(vectorList[max].begin(), vectorList[max].end()));

    auto theta = vmax.angle - vmin.angle;
    if (vmax.angle < vmin.angle) theta = 360 + vmax.angle - vmin.angle;
    if (theta > 180) {
        return {vmax.vec, vmin.vec};
    } else {
        return {{0, 0},
                {0, 0}};
    }
}