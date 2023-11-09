//
// Created by fabia on 16.06.2023.
//

#ifndef EXAMPLE_TREE_SCAN_HELPER_H
#define EXAMPLE_TREE_SCAN_HELPER_H

#include "CGALSetup.h"
#include <CGAL/Octree.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Search_traits_3.h>


//typedefs
typedef Kernel::Point_3 Point_3;
typedef CGAL::Search_traits_3<Kernel >  Traits;
typedef CGAL::Kd_tree<Traits,CGAL::Median_of_max_spread<Traits>> Kd_tree;
typedef CGAL::Octree<Kernel, std::vector<Kernel::Point_3>> Octree;


//TODO:change to Clockwise / counter clockwise?
enum boundarySide {
    BS_LEFT, BS_RIGHT
};

double cosTheta3(Kernel::Vector_3 u, Kernel::Vector_3 v);

double orientedAngleBetweenPlanes(Kernel::Plane_3 u, Kernel::Plane_3 v, Kernel::Vector_3 normalisedNormal);

int findBoundaryCell(const CGAL::Bbox_3 &bbox, const Kernel::Point_3 &origin, const std::pair<Kernel::Point_3, Kernel::Point_3> &fixPointSet,
                     boundarySide side, double minAngle);

std::vector<Kernel::Point_3 const*>
        findExtremalPoint(const Octree &tree, const std::pair<Kernel::Point_3, Kernel::Point_3> &fixPointSet,
                          boundarySide side, Kernel::Point_3 origin);

std::vector<Kernel::Point_3 const *>
        findExtremalPoint(const Kd_tree &tree, const std::pair<Kernel::Point_3, Kernel::Point_3> &fixPointSet,
                          boundarySide side, Kernel::Point_3 origin);


#endif //EXAMPLE_TREE_SCAN_HELPER_H
