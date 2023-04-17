#include <fstream>
#include "external/glm/glm.hpp"
#include "include/input_generators.h"
#include "include/config.h"
#include <vector>
#include "matplotlibcpp.h"


#define log_function_time(f, oStream) std::chrono::steady_clock::time_point logger_begin = std::chrono::steady_clock::now(); f;  std::chrono::steady_clock::time_point logger_end = std::chrono::steady_clock::now(); log <<  std::chrono::duration_cast<std::chrono::microseconds>(logger_end - logger_begin).count();

std::vector<glm::vec2>
optimizedGrahamScanVec2(std::vector<glm::vec2> &pointCloud, glm::vec2 &fixPoint, bool show = false);

int main() {
    std::ofstream log;
    log.open("log.txt", std::ios::out | std::ios::app);

    //seed = std::time(nullptr)+i;
    auto input = generateInputVec2<glm::vec2>(sample_size, seed, FPL_CONVEXHULL);

    log << sample_size << ",";
    log_function_time(auto res = optimizedGrahamScanVec2(input.pointCloud, input.fixPoint, MatPlotShow), log);
    log << "\n";
    std::cout << res[0].x << "|" << res[0].y << "\n";
    std::cout << res[1].x << "|" << res[1].y << "\n";
    std::cout << "_____________________________________________\n";
    std::cout << seed << "\n";

    log.close();
}

namespace plt = matplotlibcpp;

struct angleVec2Pair {
    double angle;
    glm::vec2 vec;

    angleVec2Pair(double a, glm::vec2 v) : angle(a), vec(v) {};

    bool operator<(const angleVec2Pair &other) const {
        return angle < other.angle;
    }
};

//TODO: Matplot doesnt work from different source file
std::vector<glm::vec2> optimizedGrahamScanVec2(std::vector<glm::vec2> &pointCloud, glm::vec2 &fixPoint, bool show) {
    std::vector<std::vector<angleVec2Pair>> vectorList(360);
    for (auto &p: pointCloud) {
        auto angle = atan2(fixPoint.y - p.y, fixPoint.x - p.x) * 180 / M_PI + 180; //TODO: vlt zu p-fixpoint ändern, damit die rotation klarer um den fixpunkt geht
        if (angle > 360)angle -= 360;
        vectorList[(int) angle].emplace_back(angle, p);
    }
    //Matplot vars
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> x1;
    std::vector<float> y1;
    std::pair<std::vector<float>, std::vector<float>> line;

    //Matplot
    if (show) {
        //convert vectors for matplot
        x1.emplace_back(fixPoint.x);
        y1.emplace_back(fixPoint.y);


        line.first.emplace_back(fixPoint.x);
        line.first.emplace_back(fixPoint.x);
        line.second.emplace_back(fixPoint.y);
        line.second.emplace_back(fixPoint.y);
    }

    int offset = -1;
    double min = 0;
    double max = 0;
    double angle = 0;
    for (int i = 0; i < vectorList.size(); i++) {
        //Matplot
        if (show) {
            for (auto &[a, v]: vectorList[i]) {
                x.emplace_back(v.x);
                y.emplace_back(v.y);
                line.first[1] = v.x;
                line.second[1] = v.y;
                //std::cout << k<<"\n";
                plt::clf();
                plt::ylim(-120, 120);
                plt::xlim(-120, 120);
                plt::plot(x, y, {{"linewidth",  "0.0"},
                                 {"marker",     "x"},
                                 {"markersize", "2.5"}});
                plt::plot(x1, y1, {{"linewidth",       "0.0"},
                                   {"marker",          "o"},
                                   {"markerfacecolor", "r"},
                                   {"markeredgecolor", "r"}});
                plt::plot(line.first, line.second, {{"linewidth", "0.5"}});

                plt::draw();
                plt::pause(0.001);
            }
        }

        if (not vectorList[i].empty()) {
            if (offset == -1) offset = i;
            if (angle > max - min) {
                min = i - angle;
                max = i;
            }
            if (angle > 180)break;
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

    //Matplot
    if (show) {
        std::pair<std::vector<float>, std::vector<float>> res;
        res.first.emplace_back(vmin.vec.x + (vmin.vec.x - fixPoint.x));
        res.first.emplace_back(fixPoint.x);
        res.first.emplace_back(vmax.vec.x + (vmax.vec.x - fixPoint.x));
        res.second.emplace_back(vmin.vec.y + (vmin.vec.y - fixPoint.y));
        res.second.emplace_back(fixPoint.y);
        res.second.emplace_back(vmax.vec.y + (vmax.vec.y - fixPoint.y));
        plt::plot(res.first, res.second, {{"linewidth", "1.5"}});

        plt::show();
    }

    auto theta = vmax.angle - vmin.angle;
    if (vmax.angle < vmin.angle) theta = 360 + vmax.angle - vmin.angle;
    if (theta > 180) {
        return {vmax.vec, vmin.vec};
    } else {
        return {{0, 0},
                {0, 0}};
    }
}