//
// Created by fabia on 01.02.2023.
//

#ifndef EXAMPLE_INPUT_GENERATORS_H
#define EXAMPLE_INPUT_GENERATORS_H

#include <vector>
#include <ctime>
#include "CGALSetup.h"
#include <CGAL/IO/polygon_soup_io.h>
#include <CGAL/Min_sphere_of_spheres_d.h>
#include <CGAL/Min_sphere_of_points_d_traits_2.h>
#include <CGAL/Min_sphere_of_points_d_traits_3.h>
#include <random>

typedef  CGAL::Min_sphere_of_spheres_d<CGAL::Min_sphere_of_points_d_traits_2<Kernel,double>>            Min_circle;
typedef  CGAL::Min_sphere_of_spheres_d<CGAL::Min_sphere_of_points_d_traits_3<Kernel,double>>            Min_sphere;

std::vector<Kernel::Point_3> generateInputVec3(int sample_size, int seed = -1,
                            size_t x_dim = 100, size_t y_dim = 100, size_t z_dim = 100);

std::vector<Kernel::Point_3> readInputVec3(std::string path);


#endif //EXAMPLE_INPUT_GENERATORS_H
