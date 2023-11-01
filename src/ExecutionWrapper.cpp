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
                std::cerr << "Help Text\n";
                exit(-1);
        }
    }

    //all parameters processed
    if(not seed and path.empty()){
        std::cerr << "seed or path required\n";
        exit(-1);
    }
    if(seed and not path.empty()){
        std::cerr << "seed and path cant be set at the same time\n";
        exit(-1);
    }
    if(AType == NoAlgorithm){
        std::cerr << "No algorithm specified. Choose between: -QH, -KD or -OT\n";
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
            std::cerr << "No shape specified. Choose between: BoxFull, BoxSurface, SphereFull, SphereSurface\n";
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
}

std::chrono::nanoseconds ExecutionWrapper::executeQuickhull() {
    prepareData();

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    CGAL::Polyhedron_3<Kernel> poly;
    CGAL::convex_hull_3(data.begin(), data.end(), poly);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    return std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
}

std::chrono::nanoseconds ExecutionWrapper::executeKdMarch() {
    prepareData();

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    Mesh res;
    KDWrap(data, res);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
}

std::chrono::nanoseconds ExecutionWrapper::executeOctreeMarch() {
    prepareData();

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    Mesh res;
    octreeWrap(data, res);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
}

std::chrono::nanoseconds ExecutionWrapper::executeGiftWrapping() {
    prepareData();

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    Mesh res;
    GiftWrap(data, res);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
}

int main(int argc, char *argv[]) {
    auto demo = ExecutionWrapper(argc,argv);
    demo.prepareData();
    switch (demo.AType) {
        case Quickhull:
            std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(demo.executeQuickhull()).count()<< "ms" << std::endl;
            break;
        case KdMarch:
            std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(demo.executeKdMarch()).count()<< "ms"<< std::endl;
            break;
        case OctreeMarch:
            std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(demo.executeOctreeMarch()).count()<< "ms"<< std::endl;
            break;
        case GiftWrapping:
            std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(demo.executeGiftWrapping()).count()<< "ms"<< std::endl;
            break;
        default:
            exit(-1);
    }
}


