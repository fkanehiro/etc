#include <hrpUtil/Eigen3d.h>
#include <iostream>

int main(int argc, char *argv[])
{
  if (argc < 10){
    std::cerr << "Usage:" << argv[0] << " R00 R01 R02 ... R22" << std::endl;
    return 1;
  }

  hrp::Matrix33 R;
  for (int i=0; i<3; i++){
    for (int j=0; j<3; j++){
      R(i,j) = atof(argv[i*3+j+1]);
    }
  }

  std::cout << "R:" << std::endl << R << std::endl; 

  hrp::Vector3 omega = hrp::omegaFromRot(R);
  double th = omega.norm();
  hrp::Vector3 axis = omega.normalized();

  for (int i=0; i<3; i++){
    std::cout << axis[i] << " ";
  }
  std::cout << th << std::endl;

  return 0;
}
