#include <iostream>
#include <fstream>
#include "SphereTreeUtil.h"
#include "SphereTree.h"

void loadPointArray(const char *i_filename, std::vector<hrp::Vector3> &o_points){
    std::ifstream ifs(i_filename);
    if (!ifs.is_open()) {
        std::cerr << "failed to open(" << i_filename << ")" << std::endl;
        return;
    }
    
    std::cerr << "loading " << i_filename << "..." << std::flush;
    hrp::Vector3 p;
    ifs >> p[0];
    while (!ifs.eof()){
        ifs >> p[1] >> p[2];
        o_points.push_back(p);
        ifs >> p[0];
    }
    std::cerr << "done(" << o_points.size() << " points)." << std::endl;
}


