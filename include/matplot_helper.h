//
// Created by fabia on 20.06.2023.
//

#ifndef EXAMPLE_MATPLOT_HELPER_H
#define EXAMPLE_MATPLOT_HELPER_H

#include <vector>

#include "CGALSetup.h"
#include <CGAL/Octree.h>

#include "../external/matplotlibcpp/matplotlibcpp.h"


typedef Kernel::Point_3 Point_3;

namespace plt = matplotlibcpp;


class matplotArray{
public:
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;

    void addPoint(Point_3 p){
        x.emplace_back(p.x());
        y.emplace_back(p.y());
        z.emplace_back(p.z());
    };
    void addList(std::vector<Point_3> &list){
        for (auto &p: list) {
            this->addPoint(p);
        }
    };
};

class matplotOctree{
private:
    std::vector<matplotArray> boxes;
    void addBox(CGAL::Bbox_3 bbox){
        boxes.emplace_back();
        boxes.back().addPoint({bbox.max(0), bbox.max(1), bbox.max(2)});
        boxes.back().addPoint({bbox.max(0), bbox.min(1), bbox.max(2)});
        boxes.back().addPoint({bbox.max(0), bbox.min(1), bbox.min(2)});
        boxes.back().addPoint({bbox.max(0), bbox.max(1), bbox.min(2)});
        boxes.back().addPoint({bbox.max(0), bbox.max(1), bbox.max(2)});

        boxes.back().addPoint({bbox.max(0), bbox.max(1), bbox.max(2)});
        boxes.back().addPoint({bbox.min(0), bbox.max(1), bbox.max(2)});
        boxes.back().addPoint({bbox.min(0), bbox.max(1), bbox.min(2)});
        boxes.back().addPoint({bbox.max(0), bbox.max(1), bbox.min(2)});
        boxes.back().addPoint({bbox.max(0), bbox.max(1), bbox.max(2)});

        boxes.back().addPoint({bbox.max(0), bbox.max(1), bbox.max(2)});
        boxes.back().addPoint({bbox.min(0), bbox.max(1), bbox.max(2)});
        boxes.back().addPoint({bbox.min(0), bbox.min(1), bbox.max(2)});
        boxes.back().addPoint({bbox.max(0), bbox.min(1), bbox.max(2)});
        boxes.back().addPoint({bbox.max(0), bbox.max(1), bbox.max(2)});


        boxes.back().addPoint({bbox.min(0), bbox.max(1), bbox.max(2)});
        boxes.back().addPoint({bbox.min(0), bbox.min(1), bbox.max(2)});
        boxes.back().addPoint({bbox.min(0), bbox.min(1), bbox.min(2)});


        boxes.back().addPoint({bbox.min(0), bbox.min(1), bbox.min(2)});
        boxes.back().addPoint({bbox.min(0), bbox.max(1), bbox.min(2)});
        boxes.back().addPoint({bbox.min(0), bbox.max(1), bbox.max(2)});
        boxes.back().addPoint({bbox.min(0), bbox.min(1), bbox.max(2)});
        boxes.back().addPoint({bbox.min(0), bbox.min(1), bbox.min(2)});

        boxes.back().addPoint({bbox.min(0), bbox.min(1), bbox.min(2)});
        boxes.back().addPoint({bbox.max(0), bbox.min(1), bbox.min(2)});
        boxes.back().addPoint({bbox.max(0), bbox.min(1), bbox.max(2)});
        boxes.back().addPoint({bbox.min(0), bbox.min(1), bbox.max(2)});
        boxes.back().addPoint({bbox.min(0), bbox.min(1), bbox.min(2)});

        boxes.back().addPoint({bbox.min(0), bbox.min(1), bbox.min(2)});
        boxes.back().addPoint({bbox.max(0), bbox.min(1), bbox.min(2)});
        boxes.back().addPoint({bbox.max(0), bbox.max(1), bbox.min(2)});
        boxes.back().addPoint({bbox.min(0), bbox.max(1), bbox.min(2)});
        boxes.back().addPoint({bbox.min(0), bbox.min(1), bbox.min(2)});
    }
public:
    matplotOctree(CGAL::Octree<Kernel, std::vector<Kernel::Point_3>> & octree){
        std::stack<CGAL::Octree<Kernel, std::vector<Kernel::Point_3>>::Node> stack;
        stack.push(octree.root());

        while(not stack.empty()){
            auto currentNode = stack.top();
            stack.pop();
            addBox(octree.bbox(currentNode));
            if (not currentNode.is_leaf()) {
                stack.push(currentNode[0]);
                stack.push(currentNode[1]);
                stack.push(currentNode[2]);
                stack.push(currentNode[3]);
                stack.push(currentNode[4]);
                stack.push(currentNode[5]);
                stack.push(currentNode[6]);
                stack.push(currentNode[7]);
            }
        }
    }
    void show(){
        for(auto &b: boxes){
            plt::plot3(b.x, b.y, b.z, {{"linewidth",  "0.5"}, {"color", "y"}}, 1);
        }
    }
};
#endif //EXAMPLE_MATPLOT_HELPER_H
