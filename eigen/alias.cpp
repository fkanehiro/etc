#include <Eigen/Core>
#include <iostream>

using namespace Eigen;

int main(int argc, char *argv[])
{
    Vector3d v1(1,2,3), v2(4,5,6);
    Matrix3d M1, M2;
    M1 << 1,2,3,4,5,6,7,8,9;
    M2 << 10,11,12,13,14,15,16,17,18;

    v1 = M1*v1 + M2*v2;
    std::cout << "without eval:" << v1 << std::endl;

    v1 << 1,2,3;

    v1 = (M1*v1 + M2*v2).eval();
    std::cout << "with eval:" << v1 << std::endl;
    return 0;
}
