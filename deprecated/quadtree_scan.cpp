#include <stack>
#include <algorithm>
#include <vector>
#include <iostream>
#include <chrono>
#include "include/input_generators.h"
#include "include/config.h"
#include "CGALSetup.h"
#include <CGAL/Quadtree.h>

//macros
#define calcDeterminate(u, v) (u.x()*v.y()-u.y()*v.x())

//types & enums
typedef CGAL::Quadtree<Kernel, std::vector<Kernel::Point_2>> Quadtree;
enum boundarySide {
    BS_LEFT, BS_RIGHT
};

//function declarations
double cosTheta(Kernel::Vector_2 u, Kernel::Vector_2 v);

bool bboxContainsPoint(const CGAL::Bbox_2 &bbox, const Kernel::Point_2 &p);

int findBoundaryQuadrant(const CGAL::Bbox_2 &bbox, const Kernel::Point_2 &origin, const Kernel::Point_2 &fixPoint,
                         boundarySide side, double minAngle = 0);

Kernel::Point_2 const *findBoundaryPoint(const Quadtree &quadtree, const Kernel::Point_2 &fixPoint, boundarySide side);

std::vector<Kernel::Point_2>
quadtreeScan(std::vector<Kernel::Point_2> &pointCloud, Kernel::Point_2 &fixPoint);

int main() {
    auto input = generateInputVec2(sample_size, seed, FPL_CONVEXHULL);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    auto res = quadtreeScan(input.pointCloud, input.fixPoint);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Time difference = "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]"
              << std::endl;
    std::cout << "Time difference = "
              << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[mircos]"
              << std::endl;
    std::cout << "Time difference = "
              << std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() << "[ns]"
              << std::endl;

    //std::cout << counter << std::endl;
    std::cout << "seed:" << seed << std::endl;

    return 0;
}

std::vector<Kernel::Point_2>
quadtreeScan(std::vector<Kernel::Point_2> &pointCloud, Kernel::Point_2 &fixPoint){
    std::vector<Kernel::Point_2> res;

    Quadtree quadtree(pointCloud);
    quadtree.refine(10, 15);

    Kernel::Point_2 const *res1 = findBoundaryPoint(quadtree, fixPoint, BS_LEFT);
    if(res1 != nullptr){
        res.emplace_back(*res1);
    }
    Kernel::Point_2 const *res2 = findBoundaryPoint(quadtree, fixPoint, BS_RIGHT);
    if(res2 != nullptr){
        res.emplace_back(*res2);
    }
    return res;
}


double cosTheta(Kernel::Vector_2 u, Kernel::Vector_2 v) {
    return u * v / (sqrt(u.squared_length()) * sqrt(v.squared_length()));
}

bool bboxContainsPoint(const CGAL::Bbox_2 &bbox, const Kernel::Point_2 &p) {
    if (bbox.xmin() < p.x() and p.x() < bbox.xmax() and bbox.ymin() < p.y() and p.y() < bbox.ymax()) return true;
    return false;
}

int findBoundaryQuadrant(const CGAL::Bbox_2 &bbox, const Kernel::Point_2 &origin, const Kernel::Point_2 &fixPoint,
                         boundarySide side, double minAngle) {
//for index order see: https://doc.cgal.org/latest/Orthtree/classCGAL_1_1Orthtree_1_1Node.html#a706069ea795fdf65b289f597ce1eb8fd

    Kernel::Vector_2 fixToOrigin(fixPoint, origin);
    Kernel::Vector_2 fixToCorner[4] = {
            {fixPoint, {bbox.xmin(), bbox.ymin()}},
            {fixPoint, {bbox.xmax(), bbox.ymin()}},
            {fixPoint, {bbox.xmin(), bbox.ymax()}},
            {fixPoint, {bbox.xmax(), bbox.ymax()}}
    };

    double det[4] = {
            calcDeterminate(fixPoint, fixToCorner[0]),
            calcDeterminate(fixPoint, fixToCorner[1]),
            calcDeterminate(fixPoint, fixToCorner[2]),
            calcDeterminate(fixPoint, fixToCorner[3])
    };

    double angle[4] = {
            acos(cosTheta(fixToOrigin, fixToCorner[0])),
            acos(cosTheta(fixToOrigin, fixToCorner[1])),
            acos(cosTheta(fixToOrigin, fixToCorner[2])),
            acos(cosTheta(fixToOrigin, fixToCorner[3]))
    };
    //give angele orientation based on sign of determinate
    if(det[0]<0) angle[0] *= -1;
    if(det[1]<0) angle[1] *= -1;
    if(det[2]<0) angle[2] *= -1;
    if(det[3]<0) angle[3] *= -1;

    int index = 0;
    double maxAngle = 0;

    switch (side) {
        case BS_LEFT:
            for (int i = 0; i < 4; i++) {
                if (maxAngle < -angle[i]) {
                    index = i;
                    maxAngle = -angle[i];
                }
            }

            if (minAngle > maxAngle) {
                auto bounds = std::minmax_element(angle, angle + 3);
                if (not(*bounds.first < 0 and 0 < *bounds.second)) index = -1;
            }
        case BS_RIGHT:
            for (int i = 0; i < 4; i++) {
                if (maxAngle < angle[i]) {
                    index = i;
                    maxAngle = angle[i];
                }
            }

            if (minAngle > maxAngle) {
                auto bounds = std::minmax_element(angle, angle + 3);
                if (not(*bounds.first < 0 and 0 < *bounds.second)) index = -1;
            }
    }
    return index;
}

Kernel::Point_2 const *findBoundaryPoint(const Quadtree &quadtree, const Kernel::Point_2 &fixPoint, boundarySide side) {
    Kernel::Point_2 const *res = nullptr;
    auto bbox = quadtree.bbox(quadtree.root());
    Kernel::Point_2 origin((bbox.xmax() + bbox.xmin()) / 2, (bbox.ymax() + bbox.ymin()) / 2);
    Kernel::Vector_2 fixToOrigin(fixPoint, origin);

    double angle = 0;
    std::vector<int> quadrantOrder = {0, 1, 3, 2};

    std::stack<Quadtree::Node> stack;
    stack.push(quadtree.root());


    while (not stack.empty()) {
        auto currentNode = stack.top();
        stack.pop();
        if (currentNode.is_leaf()) {
            for (auto const &p: currentNode) {
                double angle2 = acos(cosTheta(fixToOrigin, {fixPoint, p}));
                auto fixToP = Kernel::Vector_2(fixPoint, p);

                switch (side) {
                    case BS_LEFT:
                        if (angle < angle2 and calcDeterminate(fixToOrigin, fixToP) > 0) {
                            angle = angle2;
                            res = &p;
                        }
                        break;
                    case BS_RIGHT:
                        if (angle < angle2 and calcDeterminate(fixToOrigin, fixToP) < 0) {
                            angle = angle2;
                            res = &p;
                        }
                        break;
                }

            }
        } else {
            int quad = findBoundaryQuadrant(quadtree.bbox(currentNode), origin, fixPoint, side, angle);
            if (quad >= 0 or bboxContainsPoint(quadtree.bbox(currentNode), fixPoint)) {
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
