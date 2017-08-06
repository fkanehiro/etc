#include <hrpUtil/Eigen3d.h>
#include <iostream>

int main(int argc, char *argv[])
{
  if (argc < 5){
    std::cerr << "Usage:" << argv[0] << " Axis_X Axis_Y Axis_Z Angle" << std::endl;
    return 1;
  }

  hrp::Vector3 axis(atof(argv[1]), atof(argv[2]), atof(argv[3]));
  double angle = atof(argv[4]);
  hrp::Matrix33 R;
  hrp::calcRodrigues(R, axis, angle);
  std::cout << "R:" << std::endl << R << std::endl; 

  return 0;
}
