#include <hrpUtil/Eigen3d.h>
#include <iostream>

int main(int argc, char *argv[])
{
  if (argc < 10){
    std::cerr << "Usage:" << argv[0] << " roll pitch yaw" << std::endl;
  }

  hrp::Vector3 rpy;
  for (int i=0; i<3; i++){
    rpy[i] = atof(argv[i+1]);
  }
  hrp::Matrix33 R = hrp::rotFromRpy(rpy);

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
