//
// Created by fabia on 01.02.2023.
//

#include "input_generators.h"

// generate random number for the range (min, max)
double irand(int min, int max)
{
    return ((double)rand() / ((double)RAND_MAX + 1.0)) * (max - min) + min;
}

std::vector<Kernel::Point_3>
generateInputVec3(int sample_size, int seed, double x_dim, double y_dim, double z_dim, SHAPE shape) {
    std::vector<Kernel::Point_3> res;
    if(seed != -1) srand(seed);

    switch (shape) {
        case SphereFull:{
            // https://karthikkaranth.me/blog/generating-random-points-in-a-sphere/#using-normally-distributed-random-numbers
            //not quite uniform for an ellipsoid, only for spheres
            for (int i = 0; i < sample_size; i++) {
                double u = irand(0,1);
                double x1 = irand(-1,1);
                double x2 = irand(-1,1);
                double x3 = irand(-1,1);

                double mag = sqrt(x1*x1 + x2*x2 + x3*x3);
                x1 /= mag;
                x2 /= mag;
                x3 /= mag;

                // Math.cbrt is cube root
                double c = cbrt(u);
                res.emplace_back(x1*c*x_dim/2, x2*c*y_dim/2, x3*c*z_dim/2);
            }
            break;
        }
        case SphereSurface:{
            // https://www.bogotobogo.com/Algorithms/uniform_distribution_sphere.php
            //not quite uniform for an ellipsoid, only for spheres
            for (int i = 0; i < sample_size; i++) {
                double theta = 2*M_PI*irand(0,1);
                double phi = acos(2*irand(0,1)-1.0);

                res.emplace_back(cos(theta)*sin(phi)*x_dim/2, sin(theta)*sin(phi)*y_dim/2, cos(phi)*z_dim/2);
            }
            break;
        }
        case BoxFull:{
            //not quite uniform for cuboids, only for perfect cubes
            for (int i = 0; i < sample_size; i++) {
                double x1 = irand(-x_dim/2, x_dim/2);
                double x2 = irand(-y_dim/2, y_dim/2);
                double x3 = irand(-z_dim/2, z_dim/2);
                res.emplace_back(x1, x2, x3);
            }
            break;
        }
        case BoxSurface:{
            //not quite uniform for cuboids, only for perfect cubes
            for (int i = 0; i < sample_size; i++) {
                double x1 = irand(-x_dim/2, x_dim/2);
                double x2 = irand(-y_dim/2, y_dim/2);
                double x3 = irand(-z_dim/2, z_dim/2);
                switch (rand()%3) {
                    case 0:
                        if(rand()%2 == 0){
                            res.emplace_back(-x_dim/2, x2, x3);
                        }else{
                            res.emplace_back(x_dim/2, x2, x3);
                        }
                        break;
                    case 1:
                        if(rand()%2 == 0){
                            res.emplace_back(x1, -y_dim/2, x3);
                        }else{
                            res.emplace_back(x1, y_dim/2, x3);
                        }
                        break;
                    case 2:
                        if(rand()%2 == 0){
                            res.emplace_back(x1, x2, -z_dim/2);
                        }else{
                            res.emplace_back(x1, x2, z_dim/2);
                        }
                        break;
                }
            }
            break;
        }

    }
    return res;
}

std::vector<Kernel::Point_3> readInputVec3(std::string path) {
    std::vector<Kernel::Point_3> res;

    std::vector<std::vector<std::size_t> > polygons;

    CGAL::IO::read_polygon_soup(path, res, polygons);

    return res;
}
