#include <fstream>
#include <iostream>
#include <hrpUtil/uBlasCommonTypes.h>
#include <sys/time.h>
#include "LCPSolver.h"

using namespace hrp;

int main(int argc, char *argv[])
{
#if 0
    std::ifstream lcp("lcp.dat");
    int size;
    lcp >> size;
    dmatrix A(size,size);
    dvector b(size), x(size), solution(size);
    for (int i=0; i<size; i++){
        for (int j=0; j<size; j++){
            lcp >> A(i,j);
        }
    }
    for (int i=0; i<size; i++) lcp >> b(i);
    for (int i=0; i<size; i++) lcp >> solution(i);
#else
    int size = 500;
    dmatrix A(size, size);
    dvector b(size), x(size), solution(size);
    b.clear();
    A.clear();
    for (int i=0; i<size ;i++) A(i,i) = 1.0;
#endif

    LCPSolver solver;
    solver.initial();
    solver.setSize(b.size(), b.size(), 0);
    std::vector<int> v1;
    dvector v2(b.size());
    struct timeval tm1, tm2;
    x.clear();
    gettimeofday(&tm1, NULL);
    solver.solveMCPByProjectedGaussSeidel(A, b, v1, v2, x);
    gettimeofday(&tm2, NULL);
    std::cout << "size = " << size << ", eigen = "
#ifdef USE_EIGEN
              << "on"
#else
              << "off"
#endif 
              << ", vectorize = "
#ifdef VECTORIZE
              << "on"
#else
              << "off"
#endif
              << std::endl;
    std::cout << "time:" << (tm2.tv_sec - tm1.tv_sec)*1e3 + (tm2.tv_usec - tm1.tv_usec)/1e3 << "[ms]" << std::endl;
    //std::cout << "x:" << std::endl << x << std::endl;
    //std::cout << "solution:" << std::endl << solution << std::endl;
    
    return 0;
}
