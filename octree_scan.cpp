//
// Created by fabia on 06.03.2023.
//
#include "include/input_generators.h"
#include "include/config.h"

#include "CGALSetup.h"
#include <CGAL/Octree.h>

#include "external/matplotlibcpp/matplotlibcpp.h"

//namespaces
namespace plt = matplotlibcpp;

//typedefs
typedef Kernel::Point_3 Point_3;
typedef CGAL::Octree<Kernel, std::vector<Kernel::Point_3>> Octree;
//TODO:change to Clockwise / counter clockwise?
enum boundarySide {
    BS_LEFT, BS_RIGHT
};

//function def
Kernel::Point_3 const *findBoundaryPoint(const Octree &octree, const std::pair<Kernel::Point_3, Kernel::Point_3> &fixPointSet, boundarySide side);

std::vector<Kernel::Point_3>
octreeScan(std::vector<Kernel::Point_3> &pointCloud, std::pair<Kernel::Point_3,Kernel::Point_3> &fixPointSet);

int main() {
    auto input = generateInputVec3(sample_size, seed, FPL_CONVEXHULL);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    auto res = octreeScan(input.pointCloud, input.fixPointSet);

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

    if (MatPlotShow) {
        //Matplot
        std::vector<std::vector<double>> scatterPoints(3);
        std::vector<std::vector<double>> fixPointSet(3);

        for (auto &p: input.pointCloud) {
            scatterPoints[0].emplace_back(p.x());
            scatterPoints[1].emplace_back(p.y());
            scatterPoints[2].emplace_back(p.z());
        }

        plt::figure(1);
        plt::clf();
        plt::plot3(scatterPoints[0], scatterPoints[1], scatterPoints[2], {{"linewidth",  "0.0"},
                                                                          {"marker",     "x"},
                                                                          {"markersize", "2.5"}}, 1);

        fixPointSet[0].emplace_back(input.fixPointSet.first.x());
        fixPointSet[0].emplace_back(input.fixPointSet.second.x());
        fixPointSet[1].emplace_back(input.fixPointSet.first.y());
        fixPointSet[1].emplace_back(input.fixPointSet.second.y());
        fixPointSet[2].emplace_back(input.fixPointSet.first.z());
        fixPointSet[2].emplace_back(input.fixPointSet.second.z());

        fixPointSet[0].emplace_back(res[0].x());
        fixPointSet[0].emplace_back(input.fixPointSet.first.x());
        fixPointSet[0].emplace_back(res[1].x());
        fixPointSet[0].emplace_back(input.fixPointSet.second.x());

        fixPointSet[1].emplace_back(res[0].y());
        fixPointSet[1].emplace_back(input.fixPointSet.first.y());
        fixPointSet[1].emplace_back(res[1].y());
        fixPointSet[1].emplace_back(input.fixPointSet.second.y());

        fixPointSet[2].emplace_back(res[0].z());
        fixPointSet[2].emplace_back(input.fixPointSet.first.z());
        fixPointSet[2].emplace_back(res[1].z());
        fixPointSet[2].emplace_back(input.fixPointSet.second.z());
        plt::plot3(fixPointSet[0], fixPointSet[1], fixPointSet[2], {{"linewidth", "1.0"},
                                                                    {"color",     "g"}}, 1);
        plt::show();
    }

}

std::vector<Kernel::Point_3>
octreeScan(std::vector<Kernel::Point_3> &pointCloud, std::pair<Kernel::Point_3,Kernel::Point_3> &fixPointSet){
    std::vector<Kernel::Point_3> res;
    /** Building the Octree **/

    Octree octree(pointCloud);
    octree.refine(10, 15);

    /** Solving the Problem here **/

    Kernel::Point_3 const *res1 = findBoundaryPoint(octree, fixPointSet, BS_LEFT);
    if(res1 != nullptr){
        res.emplace_back(*res1);
    }
    Kernel::Point_3 const *res2 = findBoundaryPoint(octree, fixPointSet, BS_RIGHT);
    if(res2 != nullptr){
        res.emplace_back(*res2);
    }
    return res;
}


double cosTheta3(Kernel::Vector_3 u, Kernel::Vector_3 v) {
    return u * v / (sqrt(u.squared_length()) * sqrt(v.squared_length()));
}
double orientedAngleBetweenPlanes(Kernel::Plane_3 u, Kernel::Plane_3 v, Kernel::Vector_3 normalisedNormal){
    return atan2(
            (CGAL::cross_product(u.orthogonal_vector(),v.orthogonal_vector()))*normalisedNormal,
            u.orthogonal_vector()*v.orthogonal_vector());
}
int findBoundaryCell(const CGAL::Bbox_3 &bbox, const Kernel::Point_3 &origin, const std::pair<Kernel::Point_3, Kernel::Point_3> &fixPointSet,
                         boundarySide side, double minAngle) {

    Kernel::Plane_3 fixToOrigin(fixPointSet.first,fixPointSet.second, origin);
    Kernel::Vector_3 normal(fixPointSet.first,fixPointSet.second);
    normal /= sqrt(normal.squared_length());

    Kernel::Plane_3 fixToCorner[8] = {
            {fixPointSet.first,fixPointSet.second, {bbox.xmin(), bbox.ymin(), bbox.zmin()}},
            {fixPointSet.first,fixPointSet.second, {bbox.xmax(), bbox.ymin(), bbox.zmin()}},
            {fixPointSet.first,fixPointSet.second, {bbox.xmin(), bbox.ymax(), bbox.zmin()}},
            {fixPointSet.first,fixPointSet.second, {bbox.xmax(), bbox.ymax(), bbox.zmin()}},
            {fixPointSet.first,fixPointSet.second, {bbox.xmin(), bbox.ymin(), bbox.zmax()}},
            {fixPointSet.first,fixPointSet.second, {bbox.xmax(), bbox.ymin(), bbox.zmax()}},
            {fixPointSet.first,fixPointSet.second, {bbox.xmin(), bbox.ymax(), bbox.zmax()}},
            {fixPointSet.first,fixPointSet.second, {bbox.xmax(), bbox.ymax(), bbox.zmax()}},
    };

    //oriented angle see: https://stackoverflow.com/questions/5188561/signed-angle-between-two-3d-vectors-with-same-origin-within-the-same-plane
    double angle[8];
    for(int i = 0; i < 8; i++) {
        angle[i] = orientedAngleBetweenPlanes(fixToCorner[i],fixToOrigin,normal);
    }

    int index = 0;
    double maxAngle = 0; //TODO:rename maxAngel ?

    switch (side) {
        case BS_LEFT:
            maxAngle = M_PI;
            for (int i = 0; i < 8; i++) {
                if (maxAngle > angle[i]) {
                    index = i;
                    maxAngle = angle[i];
                }
            }

            if (minAngle < maxAngle) {
                index = -1;
            }
            break;
        case BS_RIGHT:
            maxAngle = -M_PI;
            for (int i = 0; i < 8; i++) {
                if (maxAngle < angle[i]) {
                    index = i;
                    maxAngle = angle[i];
                }
            }

            if (minAngle > maxAngle) {
                index = -1;
            }
            break;
    }
    return index;
}

Kernel::Point_3 const *findBoundaryPoint(const Octree &octree, const std::pair<Kernel::Point_3, Kernel::Point_3> &fixPointSet, boundarySide side){
    Kernel::Point_3 const *res = nullptr;

    auto bbox = octree.bbox(octree.root());
    Kernel::Point_3 origin((bbox.xmax() + bbox.xmin()) / 2, (bbox.ymax() + bbox.ymin()) / 2, (bbox.zmax() + bbox.zmin()) / 2);

    //TODO: extract from findBoundaryCell (?)
    Kernel::Plane_3 fixToOrigin(fixPointSet.first,fixPointSet.second, origin);
    Kernel::Vector_3 normal(fixPointSet.first,fixPointSet.second);
    normal /= sqrt(normal.squared_length());

    double angle = 0;
    switch (side) {
        case BS_LEFT:
            angle = M_PI;
            break;
        case BS_RIGHT:
            angle = -M_PI;
            break;
    }
    std::vector<int> quadrantOrder = {0, 1, 3, 2};

    std::stack<Octree::Node> stack;
    stack.push(octree.root());


    while (not stack.empty()) {
        auto currentNode = stack.top();
        stack.pop();
        if (currentNode.is_leaf()) {
            for (auto const &p: currentNode) {
                double angle2 = orientedAngleBetweenPlanes({fixPointSet.first,fixPointSet.second, p},fixToOrigin,normal);
                switch (side) {
                    case BS_LEFT: //find negative angle with the biggest absolute value
                        if (angle > angle2) {
                            angle = angle2;
                            res = &p;
                        }
                        break;
                    case BS_RIGHT: //find positive angle with the biggest absolute value
                        if (angle < angle2) {
                            angle = angle2;
                            res = &p;
                        }
                        break;
                }

            }
        } else {
            //for index see: https://doc.cgal.org/latest/Orthtree/classCGAL_1_1Orthtree_1_1Node.html#a706069ea795fdf65b289f597ce1eb8fd
            int idx = findBoundaryCell(octree.bbox(currentNode), origin, fixPointSet, side, angle);
            if(idx >0) {
                stack.push(currentNode[idx ^ 7]); //flip x,y,z    //opposite corner of idx
                stack.push(currentNode[idx ^ 6]); //flip y,z      //adjacent to opposite corner
                stack.push(currentNode[idx ^ 5]); //flip x,z      //adjacent to opposite corner
                stack.push(currentNode[idx ^ 3]); //flip x,y      //adjacent to opposite corner
                stack.push(currentNode[idx ^ 4]); //flip z        //adjacent to idx
                stack.push(currentNode[idx ^ 2]); //flip y        //adjacent to idx
                stack.push(currentNode[idx ^ 1]); //flip x        //adjacent to idx
                stack.push(currentNode[idx]);
            }
        }
    }
    return res;

}
