#include <fstream>
#include <iostream>
#include <Eigen/Core>

using namespace Eigen;

void GaussSeidel(const MatrixXd& A, const VectorXd& b, VectorXd& x)
{
    for(int j=0; j < A.rows(); ++j){
        double sum = 0;
        for(int k=0; k<A.cols(); ++k){
            if (j!=k) sum +=A(j,k)*x(k);
        }
        double xnew = (-b(j) - sum) / A(j,j);
        //x(j) = xnew;
        x(j) += (xnew - x(j))*0.2;
    }
}

int main(int argc, char *argv[])
{
    std::ifstream lcp("fem.dat");
    int size;
    lcp >> size;
    MatrixXd A(size,size);
    VectorXd b(size), x(size), solution(size);
    for (int i=0; i<size; i++){
        for (int j=0; j<size; j++){
            lcp >> A(i,j);
        }
    }
    for (int i=0; i<size; i++) lcp >> b(i);
    for (int i=0; i<size; i++) lcp >> solution(i);

    x.setZero();
    VectorXd d_old, d(A*x+b);
    for (int i=0; i<5000; i++){
        d_old = d;
        GaussSeidel(A, b, x);
        d = A*x + b;
        VectorXd ddiff(d-d_old); 
        std::cout << i << " " << ddiff.norm() << std::endl;
    }
    std::cout << "x:" << std::endl << x.transpose() << std::endl;
    std::cout << "solution:" << std::endl << solution.transpose() << std::endl;
    
    return 0;
}
