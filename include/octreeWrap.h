//
// Created by fabia on 22.06.2023.
//

#ifndef EXAMPLE_OCTREEWRAP_H
#define EXAMPLE_OCTREEWRAP_H

#include "octree_scan_helper.h"

#include "CGALSetup.h"
#include <CGAL/Octree.h>
#include <CGAL/Surface_mesh.h>

//typedefs
typedef Kernel::Point_3 Point_3;
typedef CGAL::Octree<Kernel, std::vector<Kernel::Point_3>> Octree;
typedef CGAL::Surface_mesh<Kernel::Point_3> Mesh;

void
octreeWrap(std::vector<Kernel::Point_3> &pointCloud, Mesh &m);

#endif //EXAMPLE_OCTREEWRAP_H
