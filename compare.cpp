//
// Created by fabia on 21.09.2023.
//
#include "tree_scan_helper.h"
#include "input_generators.h"

int main() {
    auto foo = [](int size, int seed) {
        unsigned long long time_kd = 0;
        unsigned long long time_brute = 0;
        srand(seed);
        for(int j = 0; j<=0; j++){
            auto input = generateInputVec3(size, rand(),100,100,100,SphereFull);

            Kd_tree kd_tree(input.begin(), input.end(), Kd_tree::Splitter(1));
            kd_tree.build();

            Kernel::Point_3 origin(0, 0, 0);
            for (auto &p: kd_tree) {
                origin = {origin.x() + p.x() / kd_tree.size(), origin.y() + p.y() / kd_tree.size(),
                          origin.z() + p.z() / kd_tree.size()};
            }


            const CGAL::Kd_tree_rectangle<double, Traits::Dimension> &bbox(kd_tree.bounding_box());
            std::pair<Kernel::Point_3, Kernel::Point_3> set = {
                    {bbox.max_coord(0), bbox.max_coord(1), bbox.max_coord(2)},
                    {bbox.max_coord(0), bbox.max_coord(1), bbox.min_coord(2)}};

            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

            auto a = *findBoundaryPoint(kd_tree, set, BS_RIGHT, origin);

            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

            time_kd += std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();


            Kernel::Plane_3 u(set.first, set.second, origin);

            Kernel::Vector_3 normal(set.first, set.second); //vector along rotation axis
            normal /= sqrt(normal.squared_length());
            auto angle = 0;
            auto res = 0;

            begin = std::chrono::steady_clock::now();

            for (int i = 0; i < input.size(); i++) {
                Kernel::Plane_3 v(set.first, set.second, input[i]);
                auto a2 = atan2(
                        (CGAL::cross_product(u.orthogonal_vector(), v.orthogonal_vector())) * normal,
                        u.orthogonal_vector() * v.orthogonal_vector());
                if (angle > a2) {
                    angle = a2;
                    res = i;
                }
            }

            end = std::chrono::steady_clock::now();

            time_brute += std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();

        }

        std::cout << size << ";" << time_kd
                  << ";" << time_brute  << std::endl;
    };

    //generate input
    for (int i = 100; i < 100000000; i = i * 1.5) {
        foo(i,12345);
    }
}