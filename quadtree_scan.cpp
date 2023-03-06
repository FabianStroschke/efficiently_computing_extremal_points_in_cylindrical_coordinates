#include <cstdio>
#include <stack>
#include <algorithm>
#include <ctime>
#include <vector>
#include <iostream>
#include <chrono>
#include "../external/glm/glm.hpp"
#include "include/input_generators.h"
#include "external/matplotlibcpp/matplotlibcpp.h"
#include "CGALSetup.h"
#include <CGAL/Quadtree.h>
#include <unicodeobject.h>

//macros
#define calcDeterminate(u,v) u.x()*v.y()-u.y()*v.x()

//namespaces
namespace plt = matplotlibcpp;

//types & enums
typedef CGAL::Quadtree<Kernel, std::vector<Kernel::Point_2>> Quadtree;
enum boundarySide {BS_LEFT,BS_RIGHT};

//function declarations
double cosTheta(Kernel::Vector_2 u, Kernel::Vector_2 v);

bool bboxContainsPoint(const CGAL::Bbox_2 &bbox, const Kernel::Point_2 &p);

int findBoundaryQuadrant(const CGAL::Bbox_2 &bbox, const Kernel::Point_2 &origin, const Kernel::Point_2 &fixPoint, boundarySide side, double minAngle = 0);

Kernel::Point_2 const *findBoundaryPoint(const Quadtree &quadtree, const Kernel::Point_2 &fixPoint, boundarySide side);


int main () {
    int sample_size =1000000;
    int seed = std::time(nullptr);
    auto input = generateInputVec2<Kernel::Point_2>(sample_size, seed, FPL_CONVEXHULL);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();


    Quadtree quadtree(input.pointCloud);
    quadtree.refine(10,15);

    std::chrono::steady_clock::time_point quadTreeFinish = std::chrono::steady_clock::now();



    //for index order see: https://doc.cgal.org/latest/Orthtree/classCGAL_1_1Orthtree_1_1Node.html#a706069ea795fdf65b289f597ce1eb8fd

    Kernel::Point_2 const *res = findBoundaryPoint(quadtree, input.fixPoint, BS_LEFT);
    Kernel::Point_2 const *res2 = findBoundaryPoint(quadtree, input.fixPoint, BS_RIGHT);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(quadTreeFinish - begin).count() << "[ms]" << std::endl;
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(quadTreeFinish - begin).count() << "[mircos]" << std::endl;
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (quadTreeFinish - begin).count() << "[ns]" << std::endl;

    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - quadTreeFinish).count() << "[ms]" << std::endl;
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - quadTreeFinish).count() << "[mircos]" << std::endl;
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - quadTreeFinish).count() << "[ns]" << std::endl;

    //std::cout << counter << std::endl;
    std::cout << "seed:" << seed << std::endl;


    bool show = true;
    if(show) {
        //Matplot
        std::vector<std::vector<double>> scatterPoints(2);
        std::vector<std::vector<double>> fixpoint(2);
        std::vector<std::vector<double>> resultLine(2);
        std::vector<std::vector<double>> resultLine2(2);
        std::vector<std::vector<double>> originLine(2);
        std::vector<std::vector<double>> detLine(2);


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
            resultLine2[0].emplace_back(res2->x()+(res2->x()-input.fixPoint.x()));
            resultLine2[0].emplace_back(input.fixPoint.x());
            resultLine[0].emplace_back(input.fixPoint.x());
            resultLine[0].emplace_back(res->x()+(res->x()-input.fixPoint.x()));

            resultLine2[1].emplace_back(res2->y()+(res2->y()-input.fixPoint.y()));
            resultLine2[1].emplace_back(input.fixPoint.y());
            resultLine[1].emplace_back(input.fixPoint.y());
            resultLine[1].emplace_back(res->y()+(res->y()-input.fixPoint.y()));

            auto color = "g";
            if(calcDeterminate(Kernel::Vector_2(input.fixPoint,*res),Kernel::Vector_2(input.fixPoint, *res2))>0) color = "r";

            plt::plot(resultLine2[0], resultLine2[1], {{"linewidth", "1.5"},{"color",color}});
            plt::plot(resultLine[0], resultLine[1], {{"linewidth", "1.5"},{"color", color}});
            std::cout<<res->x()*res2->y() - res->y()*res2->x()<<std::endl;
/*
            originLine[0].emplace_back(input.fixPoint.x());
            originLine[0].emplace_back(origin.x());

            originLine[1].emplace_back(input.fixPoint.y());
            originLine[1].emplace_back(origin.y());
            plt::plot(originLine[0], originLine[1], {{"linewidth", "1.5"},{"color", "g"}});

            detLine[0].emplace_back(0);
            detLine[0].emplace_back(-fixToOrigin.x());

            detLine[1].emplace_back(0);
            detLine[1].emplace_back(-fixToOrigin.y());
            plt::plot(detLine[0], detLine[1], {{"linewidth", "1.5"},{"color", "c"}});
*/
            std::stack<Quadtree::Node> stack;
            stack.push(quadtree.root());
            while(not stack.empty()){
                auto currentNode = stack.top();
                stack.pop();

                auto box = quadtree.bbox(currentNode);
                std::vector<std::vector<double>> boxLine(2);
                boxLine[0].emplace_back(box.xmin());
                boxLine[0].emplace_back(box.xmax());
                boxLine[0].emplace_back(box.xmax());
                boxLine[0].emplace_back(box.xmin());
                boxLine[0].emplace_back(box.xmin());


                boxLine[1].emplace_back(box.ymin());
                boxLine[1].emplace_back(box.ymin());
                boxLine[1].emplace_back(box.ymax());
                boxLine[1].emplace_back(box.ymax());
                boxLine[1].emplace_back(box.ymin());


                plt::plot(boxLine[0], boxLine[1], {{"linewidth", "0.5"},
                                                   {"color",     "y"}});

                if( not currentNode.is_leaf()){
                    stack.push(currentNode[0]);
                    stack.push(currentNode[1]);
                    stack.push(currentNode[2]);
                    stack.push(currentNode[3]);
                }
            }

        }
        plt::show();
    }

    return 0;
}

double cosTheta(Kernel::Vector_2 u, Kernel::Vector_2 v){
    return u*v/(sqrt(u.squared_length())*sqrt(v.squared_length()));
}

bool bboxContainsPoint(const CGAL::Bbox_2 &bbox, const Kernel::Point_2 &p){
    if(bbox.xmin()<p.x() and p.x()<bbox.xmax() and bbox.ymin()<p.y() and p.y()<bbox.ymax()) return true;
    return false;
}

int findBoundaryQuadrant(const CGAL::Bbox_2 &bbox, const Kernel::Point_2 &origin, const Kernel::Point_2 &fixPoint, boundarySide side, double minAngle){
//for index order see: https://doc.cgal.org/latest/Orthtree/classCGAL_1_1Orthtree_1_1Node.html#a706069ea795fdf65b289f597ce1eb8fd

    Kernel::Vector_2 fixToOrigin(fixPoint,origin);
    Kernel::Vector_2 fixToCorner[4] ={
            {fixPoint, {bbox.xmin(),bbox.ymin()}},
            {fixPoint, {bbox.xmax(),bbox.ymin()}},
            {fixPoint, {bbox.xmin(),bbox.ymax()}},
            {fixPoint, {bbox.xmax(),bbox.ymax()}}
    };

    double det[4] = {
            calcDeterminate(fixPoint,fixToCorner[0]),
            calcDeterminate(fixPoint,fixToCorner[1]),
            calcDeterminate(fixPoint,fixToCorner[2]),
            calcDeterminate(fixPoint,fixToCorner[3])
    };

    double angle[4] = {
            acos(cosTheta(fixToOrigin,fixToCorner[0])),
            acos(cosTheta(fixToOrigin,fixToCorner[1])),
            acos(cosTheta(fixToOrigin,fixToCorner[2])),
            acos(cosTheta(fixToOrigin,fixToCorner[3]))
    };

    int index = 0;
    double maxAngle = 0;

    switch (side) {
        case BS_LEFT:
            for(int i= 0; i<4; i++){
                if(det[i]<0 and maxAngle<angle[i]) { index = i; maxAngle = angle[i]; }
            }

            if(minAngle>maxAngle){
                auto bounds = std::minmax_element(det, det+3);
                if(not (*bounds.first < 0 and 0 < *bounds.second)) index = -1;
            }
        case BS_RIGHT:
            for(int i= 0; i<4; i++){
                if(det[i]>0 and maxAngle<angle[i]) { index = i; maxAngle = angle[i]; }
            }

            if(minAngle>maxAngle){
                auto bounds = std::minmax_element(det, det+3);
                if(not (*bounds.first < 0 and 0 < *bounds.second)) index = -1;
            }
    }
    return index;
}

Kernel::Point_2 const *findBoundaryPoint(const Quadtree &quadtree, const Kernel::Point_2 &fixPoint, boundarySide side) {
    Kernel::Point_2 const *res = nullptr;
    auto bbox = quadtree.bbox(quadtree.root());
    Kernel::Point_2 origin((bbox.xmax()+bbox.xmin())/2,(bbox.ymax()+bbox.ymin())/2);
    Kernel::Vector_2 fixToOrigin(fixPoint,origin);

    double angle = 0;
    std::vector<int> quadrantOrder = {0,1,3,2};

    std::stack<Quadtree::Node> stack;
    stack.push(quadtree.root());


    while(not stack.empty()){
        auto currentNode = stack.top();
        stack.pop();
        if(currentNode.is_leaf()){
            for(auto const &p:currentNode){
                double angle2 = acos(cosTheta(fixToOrigin,{fixPoint,p}));
                auto fixToP = Kernel::Vector_2(fixPoint,p);

                switch (side) {
                    case BS_LEFT:
                        if(angle<angle2 and calcDeterminate(fixToOrigin,fixToP)>0){
                            angle = angle2;
                            res = &p;
                        }
                        break;
                    case BS_RIGHT:
                        if(angle<angle2 and calcDeterminate(fixToOrigin,fixToP)<0){
                            angle = angle2;
                            res = &p;
                        }
                        break;
                }

            }
        }else {
            int quad = findBoundaryQuadrant(quadtree.bbox(currentNode), origin, fixPoint, side, angle);
            if(quad >= 0 or bboxContainsPoint(quadtree.bbox(currentNode),fixPoint)) {
                if (quad == 2) { quad = 3; }
                else if (quad == 3) { quad = 2; }
                else if (quad == -1) { quad = 0; }

                for (int i = 3; i >= 0; i--) {
                    stack.push(currentNode[quadrantOrder[(quad + i) % 4]]);
                }
            }
        }
    }
    return res;
}
