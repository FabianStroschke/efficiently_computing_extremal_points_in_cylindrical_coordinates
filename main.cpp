#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

#include <cstdio>

int main() {
    std::vector<int> x;
    std::vector<int> y;
    std::vector<int> z;
    std::vector<int> x2;
    std::vector<int> y2;
    std::vector<int> z2;

    for (int i = 0; i<100; i++) {
        x2.emplace_back(i);
        y2.emplace_back(i);
        z2.emplace_back(i);
        y.emplace_back(std::rand()%100);
        x.emplace_back(std::rand()%100);
        z.emplace_back(std::rand()%100);

    }
    plt::plot3(x, y,z, {{"linewidth","0.0" }, {"marker", "o"}},1);

    plt::plot3(x2, y2,z2,{},1);

    plt::draw();


    for (int i = 0; i<100; i++) {

        x2.erase(x2.begin());
        y2.erase(y2.begin());
        z2.erase(z2.begin());
        x.erase(x.begin());
        y.erase(y.begin());
        z.erase(z.begin());
        //y.emplace_back(std::rand()%100);
        //x.emplace_back(std::rand()%100);
        //z.emplace_back(std::rand()%100);
        //x[0] = std::rand()%100;
        //y[0] = std::rand()%100;
        //z[0] = std::rand()%100;

        //plt::plot(x, y, "r");
        //plt::scatter(x, y,z,2.0, {{"color", "black"},{"marker","x"}});
        //plt::plot3(x, y,z);
        //plt::scatter(x, y,z,1.0,{},1);
        plt::pause(0.05);
        plt::draw();
        //plt::close();
    }

    plt::draw();


    plt::show();
}
