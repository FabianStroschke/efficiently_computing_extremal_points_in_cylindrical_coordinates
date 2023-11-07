//
// Created by fabia on 21.09.2023.
//
#include "tree_scan_helper.h"
#include "input_generators.h"

#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>

//typedefs
typedef CGAL::Polyhedron_3<Kernel> Polyhedron_3;

int main() {
    auto foo = [](int size, int seed) {
        unsigned long long time_kd = 0;
        unsigned long long count_kd = 0;
        unsigned long long time_ot = 0;
        unsigned long long count_ot = 0;
        unsigned long long time_brute = 0;
        unsigned long long count_brute = 0;
        srand(seed);
        for(int j = 0; j<=0; j++){
            auto input = generateInputVec3(size, rand(),100,100,100,SphereSurface);

            Polyhedron_3 poly;
            CGAL::convex_hull_3(input.begin(), input.end(), poly);

            //test for kd_trees
            Kd_tree kd_tree(input.begin(), input.end(), Kd_tree::Splitter(2));
            kd_tree.build();
            Kernel::Point_3 origin(0, 0, 0);
            for (auto &p: kd_tree) {
                origin = {origin.x() + p.x() / kd_tree.size(), origin.y() + p.y() / kd_tree.size(),
                          origin.z() + p.z() / kd_tree.size()};
            }

            for(auto it = poly.facets_begin(); it != poly.facets_end(); it++){
                auto face_iterator = *it->facet_begin();
                if(face_iterator.face()->is_triangle()) {
                    Kernel::Point_3 vertices[3] = {
                            face_iterator.vertex()->point(),
                            face_iterator.next()->vertex()->point(),
                            face_iterator.next()->next()->vertex()->point()
                    };

                    std::pair<Kernel::Point_3, Kernel::Point_3> set1 = {vertices[0],vertices[1]};
                    std::pair<Kernel::Point_3, Kernel::Point_3> set2 = {vertices[1],vertices[2]};
                    std::pair<Kernel::Point_3, Kernel::Point_3> set3 = {vertices[2],vertices[0]};

                    Kernel::Point_3 a;
                    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
                    a = **findBoundaryPoint(kd_tree, set1, BS_RIGHT, origin).begin();
                    a = **findBoundaryPoint(kd_tree, set2, BS_RIGHT, origin).begin();
                    a = **findBoundaryPoint(kd_tree, set3, BS_RIGHT, origin).begin();
                    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

                    time_kd += std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();
                    count_kd += 3;
                }
            }

            //test for octrees
            Octree octree(input);
            octree.refine(50, 2);

            for(auto it = poly.facets_begin(); it != poly.facets_end(); it++){
                auto face_iterator = *it->facet_begin();
                if(face_iterator.face()->is_triangle()) {
                    Kernel::Point_3 vertices[3] = {
                            face_iterator.vertex()->point(),
                            face_iterator.next()->vertex()->point(),
                            face_iterator.next()->next()->vertex()->point()
                    };

                    std::pair<Kernel::Point_3, Kernel::Point_3> set1 = {vertices[0],vertices[1]};
                    std::pair<Kernel::Point_3, Kernel::Point_3> set2 = {vertices[1],vertices[2]};
                    std::pair<Kernel::Point_3, Kernel::Point_3> set3 = {vertices[2],vertices[0]};

                    Kernel::Point_3 a;
                    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
                    a = **findBoundaryPoint(octree, set1, BS_RIGHT, origin).begin();
                    a = **findBoundaryPoint(octree, set2, BS_RIGHT, origin).begin();
                    a = **findBoundaryPoint(octree, set3, BS_RIGHT, origin).begin();
                    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

                    time_ot += std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();
                    count_ot += 3;
                }
            }

            //test for gift wrapping
            for(auto it = poly.facets_begin(); it != poly.facets_end(); it++){
                auto face_iterator = *it->facet_begin();
                if(face_iterator.face()->is_triangle()) {
                    Kernel::Point_3 vertices[3] = {
                            face_iterator.vertex()->point(),
                            face_iterator.next()->vertex()->point(),
                            face_iterator.next()->next()->vertex()->point()
                    };

                    Kernel::Plane_3 face(vertices[0],vertices[1],vertices[2]);
                    Kernel::Vector_3 n = face.orthogonal_vector()/ sqrt(face.orthogonal_vector().squared_length());

                    Kernel::Vector_3 a1 = CGAL::cross_product(n, vertices[1]-vertices[0]);
                    a1 /= sqrt(a1.squared_length());
                    Kernel::Vector_3 a2 = CGAL::cross_product(n, vertices[2]-vertices[1]);
                    a2 /= sqrt(a1.squared_length());
                    Kernel::Vector_3 a3 = CGAL::cross_product(n, vertices[0]-vertices[2]);
                    a3 /= sqrt(a1.squared_length());

                    double angle = 1000;
                    double angle2 = 0;
                    Kernel::Point_3 *res;

                    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

                    for (auto & i : input) {
                        Kernel::Vector_3 p = i-vertices[0];

                        angle2 = a1*p / sqrt(p.squared_length());
                        if (angle > angle2) {
                            angle = angle2;
                            res = &i;
                        }
                    }

                    angle = 1000;
                    for (auto & i : input) {
                        Kernel::Vector_3 p = i-vertices[1];

                        angle2 = a2*p / sqrt(p.squared_length());
                        if (angle > angle2) {
                            angle = angle2;
                            res = &i;
                        }
                    }

                    angle = 1000;
                    for (auto & i : input) {
                        Kernel::Vector_3 p = i-vertices[2];

                        angle2 = a3*p / sqrt(p.squared_length());
                        if (angle > angle2) {
                            angle = angle2;
                            res = &i;
                        }
                    }
                    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

                    time_brute += std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();
                    count_brute += 3;
                }
            }

        }

        std::cout << size << ";" << time_kd/count_kd << ";" << time_ot/count_ot
                  << ";" << time_brute/count_brute  << std::endl;
    };

    //generate input
    for (int i = 100; i < 1000000; i = i * 2) {
        foo(i,348232);
    }
}