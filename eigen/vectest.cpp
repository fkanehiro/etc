#include <iostream>
#include <Eigen/Core>
#include <sys/time.h>

using namespace Eigen;

int main(int argc, char *argv[])
{
#define DIM 150
    MatrixXd A(DIM, DIM);
    VectorXd x(DIM),b(DIM);

    struct timeval tm1, tm2;
    gettimeofday(&tm1, NULL);
    for (int i=0; i<100; i++){
        b = A*x;
    }
    gettimeofday(&tm2, NULL);
    std::cout << "t = " << (tm2.tv_sec - tm1.tv_sec)*1e3 + (tm2.tv_usec - tm1.tv_usec)/1e3 << "[ms]" << std::endl;

    return 1;
}
