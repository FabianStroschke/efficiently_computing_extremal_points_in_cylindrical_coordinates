//
// Created by fabia on 18.06.2023.
//
#include "tree_scan_helper.h"
#include <CGAL/intersections.h>

double cosTheta3(Kernel::Vector_3 u, Kernel::Vector_3 v) {
    return u * v / (sqrt(u.squared_length()) * sqrt(v.squared_length()));
}

double DiamondAngle(double y, double x)
{
    //added 2- so its similar to atan2
    if (y >= 0)
        return (x >= 0 ? y/(x+y) : 1-x/(-x+y));
    else
        return (x < 0 ? -2-y/(-x-y) : -1+x/(x-y));
}

double orientedAngleBetweenPlanes(Kernel::Plane_3 u, Kernel::Vector_3 v_normal, Kernel::Vector_3 normalisedNormal){
    return (u.orthogonal_vector().squared_length() == 0 or v_normal.squared_length() == 0) ? 0 : DiamondAngle(
            (CGAL::cross_product(u.orthogonal_vector(),v_normal))*normalisedNormal,
            u.orthogonal_vector()*v_normal);
}

double orientedAngleBetweenPlanes(Kernel::Plane_3 u, Kernel::Plane_3 v, Kernel::Vector_3 normalisedNormal){
    return orientedAngleBetweenPlanes(u,v.orthogonal_vector(),normalisedNormal);
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
    auto v_n=fixToOrigin.orthogonal_vector();
    for(int i = 0; i < 8; i++) {
        angle[i] = orientedAngleBetweenPlanes(fixToCorner[i],v_n,normal);
    }

    int index = -1;
    auto bestAngles = std::minmax_element(angle, angle+8);
    auto sum = abs(*bestAngles.first) + abs(*bestAngles.second);
    bool difSide = *bestAngles.second >= 0 and *bestAngles.first <= 0;
    switch (side) {
        case BS_LEFT: //"smallest" angle
            if(*bestAngles.first < minAngle){
                index = bestAngles.first - angle;
            }else if(difSide and sum > 2){
                index = bestAngles.second - angle;
            }
            break;
        case BS_RIGHT: //"biggest" angle
            if(*bestAngles.second > minAngle){
                index = bestAngles.second - angle;
            }else if(difSide and sum > 2){
                index = bestAngles.first - angle;
            }
            break;
    }
    return index;
}

Kernel::Point_3 const *findBoundaryPoint(const Octree &tree, const std::pair<Kernel::Point_3, Kernel::Point_3> &fixPointSet, boundarySide side){
    Kernel::Point_3 const *res = nullptr;

    auto bbox = tree.bbox(tree.root());
    Kernel::Point_3 origin((bbox.xmax() + bbox.xmin()) / 2, (bbox.ymax() + bbox.ymin()) / 2, (bbox.zmax() + bbox.zmin()) / 2);

    //TODO: extract from findBoundaryCell (?)
    Kernel::Plane_3 fixToOrigin(fixPointSet.first,fixPointSet.second, origin);
    Kernel::Vector_3 normal(fixPointSet.first,fixPointSet.second); //vector along rotation axis
    normal /= sqrt(normal.squared_length());

    double angle = 0;
    switch (side) {
        case BS_LEFT:
            angle = 2;
            break;
        case BS_RIGHT:
            angle = -2;
            break;
    }
    std::vector<int> quadrantOrder = {0, 1, 3, 2};

    std::stack<Octree::Node> stack;
    stack.push(tree.root());
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
            int idx = findBoundaryCell(tree.bbox(currentNode), origin, fixPointSet, side, angle);

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

std::vector<Kernel::Point_3 const*>
findBoundaryPoint(const Kd_tree &tree, const std::pair<Kernel::Point_3, Kernel::Point_3> &fixPointSet,
                  boundarySide side, Kernel::Point_3 origin) {
    std::vector<Kernel::Point_3 const *> resStack;

    const CGAL::Kd_tree_rectangle<double, Traits::Dimension>& box(tree.bounding_box());

    Kernel::Plane_3 fixToOrigin(fixPointSet.first,fixPointSet.second, origin);
    Kernel::Vector_3 normal(fixPointSet.first,fixPointSet.second); //vector along rotation axis
    normal /= sqrt(normal.squared_length());

    double angle = 0;
    switch (side) {
        case BS_LEFT:
            angle = 2;
            break;
        case BS_RIGHT:
            angle = -2;
            break;
    }

    typedef std::pair<Kd_tree::Node_const_handle,CGAL::Kd_tree_rectangle<double, Traits::Dimension>> node_bbox_pair;
    std::stack<node_bbox_pair> stack;
    stack.push(node_bbox_pair(tree.root(), tree.bounding_box()));
    CGAL::Line_3<Kernel> line(fixPointSet.first, fixPointSet.second);


    while (not stack.empty()) {
        auto currentPair = stack.top();
        stack.pop();
        if (currentPair.first->is_leaf()) {
            auto node = static_cast<Kd_tree::Leaf_node_const_handle>(currentPair.first);
            for (const auto & p : *node) {
                if(p == fixPointSet.first or p == fixPointSet.second) continue;
                double angle2 = orientedAngleBetweenPlanes({fixPointSet.first,fixPointSet.second, p},fixToOrigin,normal);
                switch (side) {
                    case BS_LEFT: //find negative angle with the biggest absolute value
                        if (angle >= angle2) {
                            angle = angle2;
                            if(not resStack.empty() and not CGAL::coplanar(fixPointSet.first,fixPointSet.second,**(resStack.begin()),p)){
                                resStack.clear();
                            }
                            resStack.emplace_back(&p);
                        }
                        break;
                    case BS_RIGHT: //find positive angle with the biggest absolute value
                        if (angle <= angle2) {
                            angle = angle2;
                            if(not resStack.empty() and not CGAL::coplanar(fixPointSet.first,fixPointSet.second,**(resStack.begin()),p)){
                                resStack.clear();
                            }
                            resStack.emplace_back(&p);
                        }
                        break;
                }
            }
        } else {
            auto node = static_cast<Kd_tree::Internal_node_const_handle>(currentPair.first);
            CGAL::Bbox_3 bbox(currentPair.second.min_coord(0),currentPair.second.min_coord(1),currentPair.second.min_coord(2),
                              currentPair.second.max_coord(0),currentPair.second.max_coord(1),currentPair.second.max_coord(2));
            int idx = findBoundaryCell(bbox, origin, fixPointSet, side, angle);
            if(idx >=0) {
                CGAL::Kd_tree_rectangle<double, Traits::Dimension> bbox_upper(currentPair.second);
                CGAL::Kd_tree_rectangle<double, Traits::Dimension> bbox_lower(currentPair.second);
                node->split_bbox(bbox_lower, bbox_upper);
                //if idx corner is the max value along the cutting dimension, push lower() than upper()

                /** idx is a bit code for a corner, see here: https://doc.cgal.org/latest/Orthtree/classCGAL_1_1Orthtree_1_1Node.html#a706069ea795fdf65b289f597ce1eb8fd
                 *  Example:
                 *      idx = 6 == 110b; cutting_dimension = 1;
                 *      0 == 110b & (1<<1)
                 *      => 0 == 110b & 010b
                 *      => 0 == 010b => false
                 *      corner 6 on the y-max side of the box, therefor bbox_upper should be evaluated first aka pushed last on the stack (upper contains all y-max corners)
                 **/
                if(0==(idx&(1<<node->cutting_dimension()))){
                    stack.push(node_bbox_pair(node->upper(), bbox_upper));
                    stack.push(node_bbox_pair(node->lower(), bbox_lower));
                }else{
                    stack.push(node_bbox_pair(node->lower(), bbox_lower));
                    stack.push(node_bbox_pair(node->upper(), bbox_upper));
                }
            }
        }
    }
    return resStack;

}
