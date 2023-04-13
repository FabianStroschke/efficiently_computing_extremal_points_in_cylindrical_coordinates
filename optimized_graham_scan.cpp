#include <fstream>
#include "external/glm/glm.hpp"
#include "include/input_generators.h"
#include "include/optimizedGrahamScan.h"
#include "include/config.h"


#define log_function_time(f,oStream) std::chrono::steady_clock::time_point logger_begin = std::chrono::steady_clock::now(); f;  std::chrono::steady_clock::time_point logger_end = std::chrono::steady_clock::now(); log <<  std::chrono::duration_cast<std::chrono::microseconds>(logger_end - logger_begin).count();

int main() {
    std::ofstream log;
    log.open ("log.txt", std::ios::out | std::ios::app );

    //seed = std::time(nullptr)+i;
    auto input = generateInputVec2<glm::vec2>(sample_size, seed, FPL_RANDOM);

    log << sample_size << ",";
    log_function_time(auto res = modifiedGrahamScanVec2(input.pointCloud, input.fixPoint,false),log); //TODO output solution vectors
    log << "\n";
    std::cout<<res[0].x<<"|"<<res[0].y<<"\n";
    std::cout<<res[1].x<<"|"<<res[1].y<<"\n";
    std::cout<<"_____________________________________________\n";

    log.close();
}