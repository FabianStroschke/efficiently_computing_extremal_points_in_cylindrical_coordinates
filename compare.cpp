//
// Created by fabia on 21.09.2023.
//
#include "tree_scan_helper.h"
#include "gift_wrapping.h"
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

        long max_faces = 1000;
        long count_faces = 0;

        srand(seed);
        for(int j = 0; j<=0; j++){
            auto input = generateInputVec3(size, rand(),100,100,100,SphereFull);

            Polyhedron_3 poly;
            CGAL::convex_hull_3(input.begin(), input.end(), poly);

            //test for kd_trees
            Kd_tree kd_tree(input.begin(), input.end(), Kd_tree::Splitter(10));
            kd_tree.build();
            Kernel::Point_3 origin(0, 0, 0);
            for (auto &p: kd_tree) {
                origin = {origin.x() + p.x() / kd_tree.size(), origin.y() + p.y() / kd_tree.size(),
                          origin.z() + p.z() / kd_tree.size()};
            }
            count_faces = 0;
            for(auto it = poly.facets_begin(); it != poly.facets_end() and max_faces>=count_faces; it++){
                count_faces++;
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
                    a = **findExtremalPoint(kd_tree, set1, BS_RIGHT, origin).begin();
                    a = **findExtremalPoint(kd_tree, set2, BS_RIGHT, origin).begin();
                    a = **findExtremalPoint(kd_tree, set3, BS_RIGHT, origin).begin();
                    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

                    time_kd += std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();
                    count_kd += 3;
                }
            }

            //test for octrees
            Octree octree(input);
            octree.refine(50, 10);

            count_faces = 0;
            for(auto it = poly.facets_begin(); it != poly.facets_end() and max_faces>=count_faces; it++){
                count_faces++;
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
                    a = **findExtremalPoint(octree, set1, BS_RIGHT, origin).begin();
                    a = **findExtremalPoint(octree, set2, BS_RIGHT, origin).begin();
                    a = **findExtremalPoint(octree, set3, BS_RIGHT, origin).begin();
                    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

                    time_ot += std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();
                    count_ot += 3;
                }
            }

            //test for gift wrapping
            count_faces = 0;
            for(auto it = poly.facets_begin(); it != poly.facets_end() and max_faces>=count_faces; it++){
                count_faces++;
                auto face_iterator = *it->facet_begin();
                if(face_iterator.face()->is_triangle()) {
                    Kernel::Point_3 vertices[3] = {
                            face_iterator.vertex()->point(),
                            face_iterator.next()->vertex()->point(),
                            face_iterator.next()->next()->vertex()->point()
                    };



                    double angle = 1000;
                    double angle2 = 0;
                    Kernel::Point_3 *res;

                    std::pair<Kernel::Point_3, Kernel::Point_3> set1 = {vertices[0],vertices[1]};
                    std::pair<Kernel::Point_3, Kernel::Point_3> set2 = {vertices[1],vertices[2]};
                    std::pair<Kernel::Point_3, Kernel::Point_3> set3 = {vertices[2],vertices[0]};

                    Kernel::Plane_3 face(vertices[0],vertices[1],vertices[2]);

                    Kernel::Point_3 a;
                    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

                    a = **findExtremalPoint(input, set1, face).begin();
                    a = **findExtremalPoint(input, set2, face).begin();
                    a = **findExtremalPoint(input, set3, face).begin();

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
    for (int i = 100; i < 10000000; i = i * 2) {
        foo(i,46854321);
    }
}