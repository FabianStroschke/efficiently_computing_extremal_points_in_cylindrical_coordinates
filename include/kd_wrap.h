//
// Created by fabia on 22.06.2023.
//

#ifndef EXAMPLE_OCTREE_WRAP_H
#define EXAMPLE_OCTREEWRAP_H


#include "CGALSetup.h"
#include "tree_scan_helper.h"
#include <CGAL/Surface_mesh.h>

//typedefs
typedef Kernel::Point_3 Point_3;
typedef CGAL::Surface_mesh<Kernel::Point_3> Mesh;

void
KDWrap(std::vector<Kernel::Point_3> &pointCloud, Mesh &m);

#endif //EXAMPLE_OCTREE_WRAP_H