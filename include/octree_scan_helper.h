//
// Created by fabia on 16.06.2023.
//

#ifndef EXAMPLE_OCTREE_SCAN_HELPER_H
#define EXAMPLE_OCTREE_SCAN_HELPER_H

#include "CGALSetup.h"
#include <CGAL/Octree.h>

//typedefs
typedef Kernel::Point_3 Point_3;
typedef CGAL::Octree<Kernel, std::vector<Kernel::Point_3>> Octree;
//TODO:change to Clockwise / counter clockwise?
enum boundarySide {
    BS_LEFT, BS_RIGHT
};

double cosTheta3(Kernel::Vector_3 u, Kernel::Vector_3 v);

double orientedAngleBetweenPlanes(Kernel::Plane_3 u, Kernel::Plane_3 v, Kernel::Vector_3 normalisedNormal);

int findBoundaryCell(const CGAL::Bbox_3 &bbox, const Kernel::Point_3 &origin, const std::pair<Kernel::Point_3, Kernel::Point_3> &fixPointSet,
                     boundarySide side, double minAngle);

Kernel::Point_3 const *findBoundaryPoint(const Octree &octree, const std::pair<Kernel::Point_3, Kernel::Point_3> &fixPointSet, boundarySide side);


#endif //EXAMPLE_OCTREE_SCAN_HELPER_H
