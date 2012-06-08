#include <iostream>
#include <sys/time.h>
#include <hrpUtil/Eigen3d.h>

using namespace hrp;

int main(int argc, char *argv[])
{
    Matrix33 R;
    Vector3 rpy(0.1,0.2,0.3);
    std::cout << "correct answer:" << rpy.transpose() << std::endl;

    R = rotFromRpy(rpy);
    
    Vector3 rpy1 = rpyFromRot(R);
    std::cout << "rpyFromRot(R):" << rpy1.transpose() << std::endl;

    Vector3 rpy2 = R.eulerAngles(2,1,0);
    std::cout << "R.eulerAngles(2,1,0):" << rpy2.transpose() << std::endl;


    return 0;
}
