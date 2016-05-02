#include <Eigen/Eigen>
#include <iostream>

using namespace Eigen;

int main()
{
  int n = 10;
  MatrixXd mat(n,2);
  for (int i=0; i<n; i++){
    mat(i,0) = ((double)random())/RAND_MAX;
    mat(i,1) = ((double)random())/RAND_MAX;
  }

  MatrixXd centered = mat.rowwise() - mat.colwise().mean();
  MatrixXd cov = centered.adjoint()*centered;

  SelfAdjointEigenSolver<MatrixXd> eig(cov);

  MatrixXd ev = eig.eigenvectors();

  MatrixXd rotated = ev*mat.transpose();
  VectorXd maxCoeff = rotated.rowwise().maxCoeff();
  VectorXd minCoeff = rotated.rowwise().minCoeff();
  VectorXd center = ev.transpose()*(maxCoeff+minCoeff)/2;
  std::cout << "center:" << center.transpose() << std::endl;
  std::cout << "rotation:" << ev << std::endl;
  std::cout << "extent:"
	    << (maxCoeff-minCoeff).transpose() << std::endl;

  return 0;
}
