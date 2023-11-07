//
// Created by fabia on 27.10.2023.
//

#ifndef EXAMPLE_GIFT_WRAPPING_H
#define EXAMPLE_GIFT_WRAPPING_H

#include "CGALSetup.h"
#include "tree_scan_helper.h"
#include <CGAL/Surface_mesh.h>
#include <CGAL/Delaunay_triangulation_2.h>

//typedefs
typedef Kernel::Point_3 Point_3;
typedef CGAL::Surface_mesh<Kernel::Point_3> Mesh;
typedef CGAL::Delaunay_triangulation_2<Kernel>  Triangulation;

void GiftWrap(std::vector<Kernel::Point_3> &pointCloud, Mesh &m);
std::vector<Kernel::Point_3 const*>
findBoundaryPoint(std::vector<Kernel::Point_3> &pointCloud, const std::pair<Kernel::Point_3, Kernel::Point_3> &fixPointSet,
                  Kernel::Plane_3 face);

#endif //EXAMPLE_GIFT_WRAPPING_H
