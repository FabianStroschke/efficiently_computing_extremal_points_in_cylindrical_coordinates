#include <cstdio>
#include <stack>
#include <algorithm>
#include <ctime>
#include <vector>
#include <iostream>
#include <chrono>
#include "../external/glm/glm.hpp"
#include "include/input_generators.h"

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

#include "CGALSetup.h"
#include <CGAL/Quadtree.h>

typedef CGAL::Quadtree<Kernel, std::vector<Kernel::Point_2>> Quadtree;

double cosTheta(Kernel::Vector_2 u,Kernel::Vector_2 v){
    return u*v/(sqrt(u.squared_length())*sqrt(v.squared_length()));
}

int main () {
    int sample_size =100;
    int seed = 1676392660;//std::time(nullptr);
    auto input = generateInputVec2<Kernel::Point_2>(sample_size, seed, FPL_RANDOM);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();


    Quadtree quadtree(input.pointCloud);
    quadtree.refine(10,1);

    std::chrono::steady_clock::time_point quadTreeFinish = std::chrono::steady_clock::now();

    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(quadTreeFinish - begin).count() << "[ms]" << std::endl;
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(quadTreeFinish - begin).count() << "[mircos]" << std::endl;
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (quadTreeFinish - begin).count() << "[ns]" << std::endl;

    //find the biggest angle cw to fixpoint
    auto getMaxQuadrant_2 = [](CGAL::Bbox_2 bbox, Kernel::Point_2 &origin, Kernel::Point_2 &fixPoint, double minAngle = 0){

        //for index order see: https://doc.cgal.org/latest/Orthtree/classCGAL_1_1Orthtree_1_1Node.html#a706069ea795fdf65b289f597ce1eb8fd

        Kernel::Vector_2 fixToOrigin(fixPoint,origin);
        Kernel::Vector_2 fixToCorner[4] ={
                {fixPoint, {bbox.xmin(),bbox.ymin()}},
                {fixPoint, {bbox.xmax(),bbox.ymin()}},
                {fixPoint, {bbox.xmin(),bbox.ymax()}},
                {fixPoint, {bbox.xmax(),bbox.ymax()}}
        };

        double det[4] = {
                fixPoint.x()*fixToCorner[0].y() - fixPoint.y()*fixToCorner[0].x(),
                fixPoint.x()*fixToCorner[1].y() - fixPoint.y()*fixToCorner[1].x(),
                fixPoint.x()*fixToCorner[2].y() - fixPoint.y()*fixToCorner[2].x(),
                fixPoint.x()*fixToCorner[3].y() - fixPoint.y()*fixToCorner[3].x()
        };

        double cosAngle[4] = {
                cosTheta(fixToOrigin,fixToCorner[0]),
                cosTheta(fixToOrigin,fixToCorner[1]),
                cosTheta(fixToOrigin,fixToCorner[2]),
                cosTheta(fixToOrigin,fixToCorner[3])
        };
        double Angle[4] = {
                acos(cosAngle[0]),
                acos(cosAngle[1]),
                acos(cosAngle[2]),
                acos(cosAngle[3])
        };

          int index = 0;
        double angle = 0;
        for(int i= 0; i<4; i++){
            if(det[i]>0 and angle<Angle[i]) { index = i; angle = Angle[i]; }
        }

        if(minAngle>angle){
            return -1;
        }else{
            return index;
        }
    };


    //for index order see: https://doc.cgal.org/latest/Orthtree/classCGAL_1_1Orthtree_1_1Node.html#a706069ea795fdf65b289f597ce1eb8fd
    std::vector<int> quadrantOrder = {0,1,3,2};

    auto bbox = quadtree.bbox(quadtree.root());
    Kernel::Point_2 origin((bbox.xmax()+bbox.xmin())/2,(bbox.ymax()+bbox.ymin())/2);
    double angle = 0;

    Kernel::Point_2 const *res = nullptr;
    int counter = 0;

    std::stack<Quadtree::Node> stack;
    stack.push(quadtree.root());

    while(not stack.empty()){
        auto currentNode = stack.top();
        stack.pop();
        if(currentNode.is_leaf()){
            for(auto const &p:currentNode){
                counter++;
                double angle2 = acos(cosTheta({input.fixPoint,origin},{input.fixPoint,p}));
                std::cout << input.fixPoint.x()*p.y() - input.fixPoint.y()*p.x() << std::endl;

                if(angle<angle2){
                    angle = angle2;
                    res = &p;
                }
            }
        }else {
            counter++;
            int quad = getMaxQuadrant_2(quadtree.bbox(currentNode), origin, input.fixPoint, angle);
            if(quad >= 0) {
                if (quad == 2) { quad = 3; } else if (quad == 3) { quad = 2; }
                for (int i = 3; i >= 0; i--) {
                    stack.push(currentNode[quadrantOrder[(quad + i) % 4]]);
                }
            }
        }
    }


    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - quadTreeFinish).count() << "[ms]" << std::endl;
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - quadTreeFinish).count() << "[mircos]" << std::endl;
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - quadTreeFinish).count() << "[ns]" << std::endl;

    std::cout << counter << std::endl;
    std::cout << "seed:" << seed << std::endl;


    bool show = true;
    if(show) {
        //Matplot
        std::vector<std::vector<double>> scatterPoints(2);
        std::vector<std::vector<double>> fixpoint(2);
        std::vector<std::vector<double>> resultLine(2);

        for (auto &p: input.pointCloud) {
            scatterPoints[0].emplace_back(p.x());
            scatterPoints[1].emplace_back(p.y());
        }

        fixpoint[0].emplace_back(input.fixPoint.x());
        fixpoint[1].emplace_back(input.fixPoint.y());

        plt::clf();
        plt::ylim(-120, 120);
        plt::xlim(-120, 120);
        plt::plot(scatterPoints[0], scatterPoints[1], {{"linewidth",  "0.0"},
                                                       {"marker",     "x"},
                                                       {"markersize", "2.5"}});
        plt::plot(fixpoint[0], fixpoint[1], {{"linewidth",       "0.0"},
                                             {"marker",          "o"},
                                             {"markerfacecolor", "r"},
                                             {"markeredgecolor", "r"}});
        if(res) {
            resultLine[0].emplace_back(input.fixPoint.x());
            resultLine[0].emplace_back(res->x()+(res->x()-input.fixPoint.x()));
            resultLine[1].emplace_back(input.fixPoint.y());
            resultLine[1].emplace_back(res->y()+(res->y()-input.fixPoint.y()));
            plt::plot(resultLine[0], resultLine[1], {{"linewidth", "1.5"}});
        }
        plt::show();
    }

    return 0;
}