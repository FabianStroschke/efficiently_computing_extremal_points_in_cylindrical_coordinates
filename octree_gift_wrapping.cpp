//
// Created by fabia on 06.03.2023.
//
#include "input_generators.h"
#include "config.h"
#include "octree_wrap.h"


int main() {
    auto input = generateInputVec3(sample_size, seed);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    Mesh res;
    octreeWrap(input, res);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Time difference = "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]"
              << std::endl;
    std::cout << "Time difference = "
              << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[mircos]"
              << std::endl;
    std::cout << "Time difference = "
              << std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() << "[ns]"
              << std::endl;

    //std::cout << counter << std::endl;
    std::cout << "seed:" << seed << std::endl;

}


