#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <hrpUtil/uBlasCommonTypes.h>
#include <sys/time.h>

using namespace Eigen;
using namespace hrp;
using namespace boost::numeric::ublas;
typedef boost::numeric::ublas::matrix<double, row_major> rmdmatrix;

#ifdef USE_EIGEN
void ProjectedGaussSeidel(const MatrixXd& A, const VectorXd& b, VectorXd& x)
#else
void ProjectedGaussSeidel(const rmdmatrix& A, const dvector& b, dvector& x)
#endif
{
#ifdef USE_EIGEN
    unsigned int c = A.cols(), r = A.rows();
#else
    unsigned int c = A.size1(), r = A.size2();
#endif    
    for(int j=0; j < r; ++j){
#ifdef VECTORIZE
        double sum = A.row(j)*x - A(j,j)*x(j);
#else
        double sum = -A(j,j)*x(j);
        for(int k=0; k<c; ++k){
            sum +=A(j,k)*x(k);
        }
#endif
        double xnew = (-b(j) - sum) / A(j,j);
        if (xnew < 0) xnew = 0;
        x(j) = xnew;
    }
}

int main(int argc, char *argv[])
{
#if 1
    std::ifstream lcp("lcp.dat");
    int size;
    lcp >> size;
#if USE_EIGEN
    MatrixXd A(size,size);
    VectorXd b(size), x(size), solution(size);
#else
    rmdmatrix A(size,size);
    dvector b(size), x(size), solution(size);
#endif
    for (int i=0; i<size; i++){
        for (int j=0; j<size; j++){
            lcp >> A(i,j);
        }
    }
    for (int i=0; i<size; i++) lcp >> b(i);
    for (int i=0; i<size; i++) lcp >> solution(i);
#else
    int size = 84;
#ifdef USE_EIGEN
    MatrixXd A(size, size);
    VectorXd b(size), x(size), solution(size);
    A.setZero();
    b.setZero();
    x.setZero();
#else
    dmatrix A(size, size);
    dvector b(size), x(size), solution(size);
    A.clear();
    b.clear();
    x.clear();
#endif
    for (int i=0; i<size ;i++) A(i,i) = 1.0;
#endif

    //VectorXd x_old, d_old, d(A*x+b);
    struct timeval tm1, tm2;
    gettimeofday(&tm1, NULL);
    for (int i=0; i<5000; i++){
        //x_old = x;
        //d_old = d;
        ProjectedGaussSeidel(A, b, x);
        //d = A*x + b;
        //VectorXd diff(x-x_old);
        //VectorXd ddiff(d-d_old); 
        //std::cout << i << " " << diff.norm() << " " << ddiff.norm() << std::endl;
    }
    gettimeofday(&tm2, NULL);
    std::cout << "size:" << size << std::endl; 
    std::cout << "time:" << (tm2.tv_sec - tm1.tv_sec)*1e3 + (tm2.tv_usec - tm1.tv_usec)/1e3 << "[ms]" << std::endl;
#ifdef USE_EIGEN
    std::cout << "x:" << std::endl << x.transpose() << std::endl;
    std::cout << "solution:" << std::endl << solution.transpose() << std::endl;
#else
    std::cout << "x:" << std::endl << x << std::endl;
    std::cout << "solution:" << std::endl << solution << std::endl;
#endif
    
    return 0;
}
