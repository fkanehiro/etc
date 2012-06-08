#include <Eigen/Core>
#include <iostream>
#include <sys/time.h>

using namespace Eigen;

int main(int argc, char *argv[])
{
    Vector3d v1,v2,v3;
    Matrix3d M1;

    v1 << 1,2,3;
    v2 << 4,5,6; 
    M1 << 1,2,3,4,5,6,7,8,9;

    const int n = 10000000;

    struct timeval tm1, tm2;

    gettimeofday(&tm1, NULL);
    for (int i=0; i<n; i++){
        v3 = v1 + M1*v2;
    }
    gettimeofday(&tm2, NULL);
    
    std::cout << "v3 = v1 - M1*v2 : " << (tm2.tv_sec - tm1.tv_sec)*1000+(tm2.tv_usec - tm1.tv_usec)/1000 << "[ms]" << std::endl;
    
    gettimeofday(&tm1, NULL);
    for (int i=0; i<n; i++){
        v3 = v1;
        v3.noalias() += M1*v2;
    }
    gettimeofday(&tm2, NULL);
    
    std::cout << "v3 = v1; v3.noalias() -= M1*v2 : " << (tm2.tv_sec - tm1.tv_sec)*1000+(tm2.tv_usec - tm1.tv_usec)/1000 << "[ms]" << std::endl;
    
    return 0;
}
