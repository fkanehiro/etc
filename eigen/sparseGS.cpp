#include <iostream>
#include <Eigen/Sparse>

using namespace Eigen;

bool GaussSeidel(
    const DynamicSparseMatrix<double, RowMajor>& A,
    const VectorXd& b, VectorXd& x,
    int numIteration)
{
    if (b.size() != A.outerSize()){
        std::cout << "b.size() != A.outerSize()" << std::endl;
    }

    for(int i=0; i < numIteration; ++i){
        for(int j=0; j < A.outerSize(); ++j){
            double sum = 0, diagvalue=0;
            for(DynamicSparseMatrix<double, RowMajor>::InnerIterator it(A,j); it; ++it){
                int c = it.col();
                if (c == j){
                    diagvalue = it.value();
                }else{
                    sum += it.value()*x(c);
                }
            }
            if (!diagvalue) std::cout << "diagonal value = 0" << std::endl;
            x(j) = (-b(j) - sum) / diagvalue;
        }

    }
    return true;
}

int main(int argc, char *argv[])
{
    const int size=200;
    DynamicSparseMatrix<double, RowMajor> A(size, size);
    VectorXd b(size), x(size), solution(size);

    for (int i=0; i<size; i++){
        solution(i) = i+1;
        A.coeffRef(i,i) = i+1;
    }
    float ratio;
    do{
        int r = ((float)random())*size/RAND_MAX;
        int c = ((float)random())*size/RAND_MAX;
        if (r == c) continue;
        A.coeffRef(r,c) = (2.9*random())/RAND_MAX;
        ratio = ((float)A.nonZeros())/(A.rows()*A.cols());
    }while(ratio<0.3);

    b = -A*solution;
    std::cout << "solution = " << x.transpose() << std::endl;
    std::cout << "A:" << std::endl << A << std::endl;
    std::cout << "b:" << b.transpose() << std::endl;

    x.setZero();
    int i=0;
    double err;
    do{
        GaussSeidel(A, b, x,1);
        VectorXd errv = x - solution;
        err = errv.norm();
        std::cout << i++ << ":" << err << std::endl;
    }while(err > 1e-3);
    
    return 0;
}
