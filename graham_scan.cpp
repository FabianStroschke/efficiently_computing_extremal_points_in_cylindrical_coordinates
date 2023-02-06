#include <cstdio>
#include <stack>
#include <algorithm>
#include <ctime>
#include <vector>
#include <iostream>
#include <chrono>
#include "../external/glm/glm.hpp"
#include "include/input_generators.h"

#include "CGALSetup.h"
#include <CGAL/ch_graham_andrew.h>

using namespace std;

int main () {
    int sample_size =50;
    int seed = std::time(nullptr);
    auto input = generateInputVec2<Kernel::Point_2>(sample_size, seed, FPL_CONVEXHULL);
    input.pointCloud.push_back(input.fixPoint); //add fixpoint to point cloud, so that graham scan can find a convex hull containing the fixpoint


    std::vector<Kernel::Point_2>  out;


    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    CGAL::ch_graham_andrew( input.pointCloud.begin(), input.pointCloud.end(), std::back_inserter(out) );

    //TODO: find solution vectors
    for(int i = 0; i< out.size(); i++){
        std::cout << out[i].x() << "|" << out[i].y() << "\n";

        if(out[i].x() == input.fixPoint.x() and out[i].y() == input.fixPoint.y()){
            //(i+out.size()-1)%out.size() enables looping around the list
            std::cout <<  "---------------------------------------" << "\n";
            std::cout << out[(i+out.size()-1)%out.size()].x() << "|" << out[(i+out.size()-1)%out.size()].y() << "\n";
            std::cout << input.fixPoint.x() << "|" << input.fixPoint.y() << "\n";
            std::cout << out[(i+out.size()+1)%out.size()].x() << "|" << out[(i+out.size()+1)%out.size()].y() << "\n";
            std::cout <<  "---------------------------------------" << "\n";

            //break;
        }
    }




    end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[mircos]" << std::endl;
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;


    return 0;
}