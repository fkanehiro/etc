#include <iostream>
#include <Eigen/Cholesky>

using namespace Eigen;

int main(int argc, char *argv[])
{
#define DIM 1000
  std::cout << "initial state" << std::flush; getchar();
  MatrixXd A(DIM,DIM);
  VectorXd b(DIM), x(DIM);
  A.setZero();
  std::cout << "A is allocated" << std::flush; getchar();
  b.setZero();
  for (int i=0; i<DIM; i++) {
    A(i,i)=1.0;
  }

  LDLT<MatrixXd> ldlt(A);
  std::cout << "ldlt is computed" << std::flush; getchar();
  for (int i=0; i<DIM; i++){
    std::cout << i << "/" << DIM << std::endl;
    b(i)=1.0;
    ldlt.solve(b, &x);
    b(i)=0.0;
    A.block(0,i,DIM,1) = x;
  }

  std::cout << "inverse of A:" << A << std::endl;
  return 1;
}
