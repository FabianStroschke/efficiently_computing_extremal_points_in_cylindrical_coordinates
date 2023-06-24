//
// Created by fabia on 18.06.2023.
//
#include "octree_scan_helper.h"
#include <CGAL/intersections.h>

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
    Kernel::Vector_3 normal(fixPointSet.first,fixPointSet.second); //vector along rotation axis
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
    double maxAngle = 0;

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
    Kernel::Vector_3 normal(fixPointSet.first,fixPointSet.second); //vector along rotation axis
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
    CGAL::Line_3<Kernel> line(fixPointSet.first, fixPointSet.second);


    while (not stack.empty()) {
        auto currentNode = stack.top();
        stack.pop();
        if (currentNode.size() == 0) continue;
        if (currentNode.is_leaf()) {
            for (auto const &p: currentNode) {
                if(p == fixPointSet.first or p == fixPointSet.second) continue;
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
            int idx = 0;
            if(not CGAL::intersection(line, octree.bbox(currentNode))){
                idx = findBoundaryCell(octree.bbox(currentNode), origin, fixPointSet, side, angle);
            }
            if(idx >=0) {
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
