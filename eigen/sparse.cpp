#include <iostream>
#include <Eigen/Sparse>
#include <sys/time.h>

using namespace Eigen;

#define USE_SPARSE

int main(int argc, char *argv)
{
#ifdef USE_SPARSE
    DynamicSparseMatrix<double> mat;
#else
    MatrixXd mat;
#endif

    const int dim=1000;
    mat.resize(dim, dim);
    mat.setZero();
    for (int i=0; i<dim; i++){
#ifdef USE_SPARSE
        mat.coeffRef(i,i) = 1.0;
#else
        mat(i,i) = 1.0;
#endif
    }
#if 1
    DynamicSparseMatrix<double> sm,sm2;
    MatrixXd dm,dm2;
#endif

    //std::cout << "mat =" << mat  << std::endl;
    VectorXd v(dim);
    for (int i=0; i<dim; i++){
        v(i) = i; 
    }

    //std::cout << "m3 = " << m3 << std::endl;

    //std::cout << "v = " << v << std::endl;
    const int loop=10000;
    struct timeval t1, t2;
    VectorXd ret(dim);
    gettimeofday(&t1, NULL);
    for (int i=0; i<loop; i++){
        ret = mat*v;
    }
    gettimeofday(&t2, NULL);
    std::cout << "mat*v = " << mat*v << std::endl;    gettimeofday(&t2, NULL);
    std::cout << (t2.tv_sec - t1.tv_sec)*1000 + (t2.tv_usec - t1.tv_usec)/1000
              << "[ms]" << std::endl;
}
