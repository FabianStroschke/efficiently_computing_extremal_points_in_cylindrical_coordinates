//
// Created by fabia on 20.06.2023.
//

#ifndef EXAMPLE_MATPLOT_HELPER_H
#define EXAMPLE_MATPLOT_HELPER_H

#include <vector>

#include "CGALSetup.h"
#include <CGAL/Octree.h>

#include <CGAL/Kd_tree.h>
#include <CGAL/Search_traits_3.h>

#include "../external/matplotlibcpp/matplotlibcpp.h"


typedef Kernel::Point_3 Point_3;
typedef CGAL::Search_traits_3<Kernel >  Traits;
typedef CGAL::Kd_tree<Traits> Kd_tree;

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

class matplotKDtree{
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
    matplotKDtree(Kd_tree & tree){

        typedef std::pair<Kd_tree::Node_const_handle,CGAL::Kd_tree_rectangle<double, Traits::Dimension>> node_bbox_pair;
        std::stack<node_bbox_pair> stack;
        stack.push(node_bbox_pair(tree.root(), tree.bounding_box()));

        while (not stack.empty()) {
            auto currentPair = stack.top();
            stack.pop();
            if (currentPair.first->is_leaf()) {
                addBox({currentPair.second.min_coord(0), currentPair.second.min_coord(1),
                        currentPair.second.min_coord(2),
                        currentPair.second.max_coord(0), currentPair.second.max_coord(1),
                        currentPair.second.max_coord(2)});
            } else {
                auto node = static_cast<Kd_tree::Internal_node_const_handle>(currentPair.first);
                CGAL::Bbox_3 bbox(currentPair.second.min_coord(0),currentPair.second.min_coord(1),currentPair.second.min_coord(2),
                                  currentPair.second.max_coord(0),currentPair.second.max_coord(1),currentPair.second.max_coord(2));
                CGAL::Kd_tree_rectangle<double, Traits::Dimension> bbox_upper(currentPair.second);
                CGAL::Kd_tree_rectangle<double, Traits::Dimension> bbox_lower(currentPair.second);
                node->split_bbox(bbox_lower, bbox_upper);

                stack.push(node_bbox_pair(node->upper(), bbox_upper));
                stack.push(node_bbox_pair(node->lower(), bbox_lower));

                addBox({currentPair.second.min_coord(0), currentPair.second.min_coord(1),
                        currentPair.second.min_coord(2),
                        currentPair.second.max_coord(0), currentPair.second.max_coord(1),
                        currentPair.second.max_coord(2)});

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
