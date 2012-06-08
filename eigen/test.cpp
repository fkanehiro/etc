#include <iostream>
#include <Eigen/Sparse>

using namespace Eigen;

int main(int argc, char *argv[])
{
#define DIM 1000
  DynamicSparseMatrix<double> m(DIM, DIM);
  for (int i=0; i<DIM; i++){
    m.coeffRef(i,i) = i;
  }

  MatrixXd m2(10,10), inv;
  bool ret;
  inv = m2.inverse();

  return 1;
}
