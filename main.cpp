#include <ctime>
#include <chrono>
#include "matplotlibcpp.h"
#include "glm/glm.hpp"
namespace plt = matplotlibcpp;

#define showMatPlot false


double graham_scan(glm::vec2 p0, glm::vec2 a, glm::vec2 b){
    //glm::mat3x3 mat = {{1.0,1.0,1.0},{},{}}
}
std::vector<std::pair<double, glm::vec2>> generateSortedAngleMap(const std::vector<glm::vec2> &points,const glm::vec2 &fixpoint){
    std::vector<std::pair<double, glm::vec2>> list;
    glm::vec2 x_axis = glm::vec2(1,0);
    for(auto &p: points){
        auto dot_angle = glm::acos(glm::dot(glm::normalize(fixpoint-p),x_axis))*360/(2*M_PI);
        if(p.y>fixpoint.y){
            dot_angle = 360-dot_angle;
        }
        list.emplace_back(dot_angle,p);
    }
    return list;
}

int main() {
    std::vector<glm::vec2> points;
    int sample_size =1000000;
    int seed = 1234;//std::time(nullptr);
    std::srand(seed);
    //generate point cloud and fixpoint
    int rand =std::rand();
    for (int i = 0; i<sample_size; i++) {
        points.emplace_back(sin(std::rand()) * 50, cos(std::rand()) * 50);
    }
    glm::vec2 fixPoint = glm::vec2(sin(rand)*100,cos(rand)*100);//(std::rand()%2000)/10.0,(std::rand()%2000)/10.0);

    auto map = generateSortedAngleMap(points,fixPoint);
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::sort(map.begin(),map.end(),[](std::pair<double, glm::vec2> &a, std::pair<double, glm::vec2> &b) {return a.first > b.first; });
    end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[mircos]" << std::endl;

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
    double min =map.begin()->first;
    double max =map.begin()->first;
    double last =map.begin()->first;
    double angle =max-min;
    for(auto [k,v]: map){
#if showMatPlot
        x.emplace_back(v.x);
        y.emplace_back(v.y);
        line.first[1]=v.x;
        line.second[1]=v.y;
        //std::cout << k<<"\n";
        plt::clf();
        plt::ylim(-120,120);
        plt::xlim(-120,120);
        plt::plot(x, y, {{"linewidth","0.0" }, {"marker", "x"},{"markersize", "2.5"}});
        plt::plot(x1,y1,{{"linewidth","0.0" }, {"marker", "o"},{"markerfacecolor","r"},{"markeredgecolor","r"}});
        plt::plot(line.first,line.second,{{"linewidth","0.5" }});

        plt::draw();
        //plt::pause(0.001);
#endif
        if(k-last>angle) {
            angle = k - last;
            min = last;
            max = k;
        }
        last = k;
    }
    if(map.begin()->first-(last-360)>angle) {
        angle = map.begin()->first-(last-360);
        min = map.begin()->first;
        max = last;
    }
    end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[mircos]" << std::endl;
#if showMatPlot
   /* std::pair<std::vector<float>,std::vector<float>> res;
    res.first.emplace_back(map.find(min)->second.x+(map.find(min)->second.x-fixPoint.x));
    res.first.emplace_back(fixPoint.x);
    res.first.emplace_back(map.find(max)->second.x+(map.find(max)->second.x-fixPoint.x));
    res.second.emplace_back(map.find(min)->second.y+(map.find(min)->second.y-fixPoint.y));
    res.second.emplace_back(fixPoint.y);
    res.second.emplace_back(map.find(max)->second.y+(map.find(max)->second.y-fixPoint.y));
    plt::plot(res.first,res.second,{{"linewidth","1.5" }});
*/
    plt::show();
#endif

    for(auto [k,v]: map){
        if(k == min or k == max){
            std::cout << k << "\n";
            break;
        }
    }
    std::cout << angle << "\n";
    std::cout << fixPoint.x << "|" << fixPoint.y << "\n";
    std::cout << seed << "\n";

    end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[mircos]" << std::endl;
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;
}
