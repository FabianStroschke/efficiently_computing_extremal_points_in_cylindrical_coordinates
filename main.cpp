#include <ctime>
#include <chrono>
#include "matplotlibcpp.h"
#include "glm/glm.hpp"
namespace plt = matplotlibcpp;

#define showMatPlot true
#define showTimeStamps false

struct angleVec2Pair{
    double angle;
    glm::vec2 vec;

    angleVec2Pair(double a, glm::vec2 v) : angle(a), vec(v){};

    bool operator<(const angleVec2Pair &other) const
    {
        return angle < other.angle;
    }
};

template<typename PointType>
struct Input{
    std::vector<PointType> pointCloud;
    PointType fixPoint{};
};

void modifiedGrahamScanVec2(std::vector<glm::vec2> pointCloud, glm::vec2 fixPoint);

enum fixPointLocation {FPL_CONVEXHULL, FPL_RANDOM, FPL_USUALLY_INSIDE, FPL_CENTER};

Input<glm::vec2> generateInputVec2(int sample_size, int seed = std::time(nullptr),fixPointLocation hint = FPL_RANDOM, size_t x_dim = 100, size_t y_dim = 100) {
    Input<glm::vec2> res;

    //generate point cloud and fixpoint
    srand(seed);
    int r = std::rand();
    double scale = 1;

    switch (hint) {
        case FPL_CONVEXHULL:
            res.fixPoint = glm::vec2(sin(r) * x_dim, cos(r) * y_dim);
            scale = 0.7;
            break;
        case FPL_RANDOM:
            res.fixPoint = glm::vec2((float )(rand() % (x_dim * 2)) - x_dim, (float )(rand() % (y_dim * 2)) - y_dim);
            break;
        case FPL_USUALLY_INSIDE:
            res.fixPoint = glm::vec2(sin(r) * x_dim/2, cos(r) * y_dim/2);
            break;
        case FPL_CENTER:
            res.fixPoint = glm::vec2(0,0);
            break;
    }

    for (int i = 0; i<sample_size; i++) {
        r = rand();
        res.pointCloud.emplace_back(sin(r) * (rand()%x_dim)*scale, cos(r) * (rand()%y_dim)*scale);
    }
    return res;
}



void modifiedGrahamScanVec2(std::vector<glm::vec2> pointCloud, glm::vec2 fixPoint) {
#if showTimeStamps
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
#endif

    std::vector<std::vector<angleVec2Pair>> vectorList(360);
    for(auto &p: pointCloud){
        auto angle = atan2(fixPoint.y-p.y,fixPoint.x-p.x)*180/M_PI+180;
        vectorList[(int)angle].emplace_back(angle,p);
    }

#if showTimeStamps
    end = std::chrono::steady_clock::now();
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[mircos]" << std::endl;
#endif
#if showMatPlot
//convert vectors for matplot
    std::vector<float> x1;
    std::vector<float> y1;
    x1.emplace_back(fixPoint.x);
    y1.emplace_back(fixPoint.y);
    plt::draw();

    std::vector<float> x;
    std::vector<float> y;
    std::pair<std::vector<float>,std::vector<float>> line;
    line.first.emplace_back(fixPoint.x);
    line.first.emplace_back(fixPoint.x);
    line.second.emplace_back(fixPoint.y);
    line.second.emplace_back(fixPoint.y);
#endif
//  plt::ion();
// plt::plot(x, y, {{"linewidth","0.0" }, {"marker", "o"}});
    int offset = -1;
    double min =0;
    double max =0;
    double angle =0;
    for(int i= 0; i<vectorList.size(); i++){
        #if showMatPlot
            for(auto &[a,v]:vectorList[i]) {
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
        #endif
        if(not vectorList[i].empty()){
            if(offset == -1) offset = i;
            if(angle>max-min) {
                min = i - angle;
                max = i;
            }
            angle = 0;
        }
        angle++;

    }
    if(angle+offset>max-min) {
        min = vectorList.size()-angle;
        max = offset;
    }
    angleVec2Pair vmin = *(std::max_element(vectorList[min].begin(),vectorList[min].end()));
    angleVec2Pair vmax = *(std::min_element(vectorList[max].begin(),vectorList[max].end()));

#if showTimeStamps
    end = std::chrono::steady_clock::now();
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[mircos]" << std::endl;
#endif
#if showMatPlot
    std::pair<std::vector<float>,std::vector<float>> res;
    res.first.emplace_back(vmin.vec.x+(vmin.vec.x-fixPoint.x));
    res.first.emplace_back(fixPoint.x);
    res.first.emplace_back(vmax.vec.x+(vmax.vec.x-fixPoint.x));
    res.second.emplace_back(vmin.vec.y+(vmin.vec.y-fixPoint.y));
    res.second.emplace_back(fixPoint.y);
    res.second.emplace_back(vmax.vec.y+(vmax.vec.y-fixPoint.y));
    plt::plot(res.first,res.second,{{"linewidth","1.5" }});

    plt::show();
#endif

    auto theta = vmax.angle-vmin.angle;
    if(vmax.angle<vmin.angle) theta = 360 +vmax.angle-vmin.angle;
    std::cout << vmax.angle<< "|" <<vmin.angle << "\n";
    std::cout << "Angle:" <<theta<< "\n";
    std::cout << fixPoint.x << "|" << fixPoint.y << "\n";
#if showTimeStamps
    end = std::chrono::steady_clock::now();
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[mircos]" << std::endl;
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;
#endif
}


int main() {
    int sample_size =100;
    int seed = std::time(nullptr);

    Input<glm::vec2> input = generateInputVec2(sample_size, seed, FPL_RANDOM);
    modifiedGrahamScanVec2(input.pointCloud, input.fixPoint); //TODO output solution vectors


    std::cout << seed << "\n";
}