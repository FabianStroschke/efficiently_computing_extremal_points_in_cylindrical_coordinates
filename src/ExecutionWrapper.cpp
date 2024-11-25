//
// Created by fabia on 24.08.2023.
//

#include "ExecutionWrapper.h"

void ExecutionWrapper::readInput(int argc, char* argv[]){
    for(int i = 0; i<argc; i++){
        if (argv[i] == std::string("-QH")){
            if(AType != NoAlgorithm){std::cerr << "Select only one algorithm.\n";exit(-1);}
            AType = Quickhull;
        }else if(argv[i] == std::string("-OT")){
            if(AType != NoAlgorithm){std::cerr << "Select only one algorithm.\n";exit(-1);}
            AType = OctreeMarch;
        }else if(argv[i] == std::string("-KD")){
            if(AType != NoAlgorithm){std::cerr << "Select only one algorithm.\n";exit(-1);}
            AType = KdMarch;
        }else if(argv[i] == std::string("-GW")){
            if(AType != NoAlgorithm){std::cerr << "Select only one algorithm.\n";exit(-1);}
            AType = GiftWrapping;
        }else if(argv[i] == std::string("-seed")){
            if(i+1< argc){
                i++;
                seed = std::atoi(argv[i]);
            } else {
                std::cerr << "Not enough arguments after '-seed'\n";
            }
        }else if(argv[i] == std::string("-size")){
            if(i+1< argc){
                i++;
                size = std::atoi(argv[i]);
            } else {
                std::cerr << "Not enough arguments after '-size'\n";
            }
        }else if(argv[i] == std::string("-dim")){
            if(i+3< argc){
                dim[0] = std::atof(argv[i+1]);
                dim[1] = std::atof(argv[i+2]);
                dim[2] = std::atof(argv[i+3]);
                i+=3;
            } else {
                std::cerr << "Not enough arguments after '-dim'\n";
            }
        }else if(argv[i] == std::string("-path")){
            if(i+1< argc){
                i++;
                path = argv[i];
            } else {
                std::cerr << "Not enough arguments after '-path'\n";
            }
        }else if(argv[i] == std::string("-shape")){
            if(i+1< argc){
                i++;
                if(argv[i] == std::string("BoxFull")) { shape = BoxFull; }
                else if (argv[i] == std::string("BoxSurface")) { shape = BoxSurface; }
                else if (argv[i] == std::string("SphereFull")) { shape = SphereFull; }
                else if (argv[i] == std::string("SphereSurface")) { shape = SphereSurface; }
                else {
                    std::cerr << "Unknown shape. Choose between: BoxFull, BoxSurface, SphereFull, SphereSurface\n";
                    exit(-1);
                }
            }
        }else if(argv[i] == std::string("-?") or argv[i] == std::string("?") or argv[i] == std::string("-help")){
                std::cerr << "Result: <n>;<h>;<t1>;<t2>\n"
                             "\tn: Number of Points\n"
                             "\th: Number of points on the Convex Hull\n"
                             "\tt1: Time to build data structure in mirco seconds\n"
                             "\tt2: Time to calculate Convex Hull in mirco seconds\n\n"
                             "How to use: \n\n"
                             "-QH\n"
                             "\tUse Quickhull as algorithm\n"
                             "\n"
                             "-OT\n"
                             "\tUse OctreeWrap as algorithm\n"
                             "\n"
                             "-KD\n"
                             "\tUse KDWrap as algorithm\n"
                             "\n"
                             "-GW\n"
                             "\tUse Gift-wrapping as algorithm\n"
                             "\n"
                             "-path <string>\n"
                             "\tPath to a file that contains a point cloud\n"
                             "\n"
                             "-seed <integer>\n"
                             "\tSeed for generation of a random point cloud\n"
                             "\n"
                             "-size <integer>\n"
                             "\tHow many points will be generated\n"
                             "\n"
                             "-dim <x> <y> <z>\n"
                             "\tThe maximum distance between 2 points along one axis\n"
                             "\tDefault dimensions x=100, y=100 z=100\n"
                             "\n"
                             "-shape <options below>\n"
                             "\tOptions: BoxFull, BoxSurface, SphereFull, SphereSurface\n"
                             "\tBoxFull, SphereFull: \n"
                             "\t\tGenerate points on the surface and the inside of the shape\n"
                             "\tBoxSurface, SphereSurface: \n"
                             "\t\tGenerate points only on the surface the shape \n"
                             "\tInfo:\n"
                             "\t\tWith 'SphereSurface' all point become part of the convex hull\n"
                             "\t\tWith 'BoxSurface' points form faces with many coplanar points\n"
                             "\n"
                             "Example use: -seed 1234567 -OT -size 100000 -shape SphereFull";
                exit(-1);
        }
    }

    //all parameters processed
    if(not seed and path.empty()){
        std::cerr << "seed or path required. Use ?, -? or -help for more info\n";
        exit(-1);
    }
    if(seed and not path.empty()){
        std::cerr << "seed and path cant be set at the same time. Use ?, -? or -help for more info\n";
        exit(-1);
    }
    if(AType == NoAlgorithm){
        std::cerr << "No algorithm specified. Choose between: -QH, -OT, -KD or -GW. Use ?, -? or -help for more info\n";
        exit(-1);
    }
    if(seed){
        if(size == 0){
            std::cerr << "size needs to be set to n > 0\n";
            exit(-1);
        }
        if(not (dim[0] or dim[1] or dim[2])){
            dim[0] = 100;
            dim[1] = 100;
            dim[2] = 100;
        }
        if(shape == NoShape){
            std::cerr << "No shape specified. Choose between: BoxFull, BoxSurface, SphereFull, SphereSurface. Use ?, -? or -help for more info\n";
            exit(-1);
        }
    }
}

void ExecutionWrapper::prepareData() {
    if(seed){
        data = generateInputVec3(this->size, this->seed, dim[0], dim[1], dim[2], shape);
    }else{
        data = readInputVec3(path);
    }
    inputSize = data.size();
}

std::chrono::nanoseconds ExecutionWrapper::executeQuickhull() {
    prepareData();

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    CGAL::Polyhedron_3<Kernel> poly;
    CGAL::convex_hull_3(data.begin(), data.end(), poly);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    outputSize = poly.points().size();
    datastructureBuildTime = std::chrono::duration_cast<std::chrono::nanoseconds>(end - end);
    convexBuildTime = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
    return std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
}

std::chrono::nanoseconds ExecutionWrapper::executeKdMarch() {
    prepareData();

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    Kd_tree kd_tree(data.begin(),data.end(),Kd_tree::Splitter(2));
    kd_tree.build();

    std::chrono::steady_clock::time_point endData = std::chrono::steady_clock::now();

    Mesh res;
    KDWrap(kd_tree, res);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    outputSize = res.vertices().size();
    datastructureBuildTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endData - begin);
    convexBuildTime = std::chrono::duration_cast<std::chrono::nanoseconds>(end - endData);
    return std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
}

std::chrono::nanoseconds ExecutionWrapper::executeOctreeMarch() {
    prepareData();

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    Octree octree(data);
    octree.refine(50, 2);

    std::chrono::steady_clock::time_point endData = std::chrono::steady_clock::now();

    Mesh res;
    octreeWrap(octree, res);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    outputSize = res.vertices().size();
    datastructureBuildTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endData - begin);
    convexBuildTime = std::chrono::duration_cast<std::chrono::nanoseconds>(end - endData);

    return std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
}

std::chrono::nanoseconds ExecutionWrapper::executeGiftWrapping() {
    prepareData();

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    Mesh res;
    GiftWrap(data, res);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    outputSize = res.vertices().size();
    datastructureBuildTime = std::chrono::duration_cast<std::chrono::nanoseconds>(end - end);
    convexBuildTime = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
    return std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
}

int main(int argc, char *argv[]) {
    auto demo = ExecutionWrapper(argc,argv);
    demo.prepareData();
    switch (demo.AType) {
        case Quickhull:
            demo.executeQuickhull();
            break;
        case KdMarch:
            demo.executeKdMarch();
            break;
        case OctreeMarch:
            demo.executeOctreeMarch();
            break;
        case GiftWrapping:
            demo.executeGiftWrapping();
            break;
        default:
            exit(-1);
    }
    std::cout << demo.inputSize <<";"<< demo.outputSize <<";"
              << std::chrono::duration_cast<std::chrono::microseconds>(demo.datastructureBuildTime).count() <<";"
              << std::chrono::duration_cast<std::chrono::microseconds>(demo.convexBuildTime).count() << std::endl;
}

