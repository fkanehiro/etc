#include <iostream>
#include <sys/time.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpUtil/TimeMeasure.h>

using namespace hrp;

int main(int argc, char *argv[])
{
    if (argc < 2){
        std::cerr << "Usage:" << argv[0] << " [url]" << std::endl;
        return 1;
    }

    BodyPtr body = new Body();
    loadBodyFromModelLoader(body, argv[1], argc, argv);

    TimeMeasure tm;
    tm.begin();
    for (int i=0; i<10000000; i++){
        body->calcForwardKinematics();
    }
    tm.end();
    std::cout << tm.totalTime() << "[ms]" << std::endl;

    return 0;
}
