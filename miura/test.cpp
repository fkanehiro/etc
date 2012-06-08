#include <iostream>
#include<Interpolator/MotionInterpolator.h>

using namespace motion_interpolator;

int main(int argc, char *argv[])
{
  MotionInterpolator mi(POINT_TO_POINT);

    double f,  x=0,v,a;
    for (int i=0; i<100; i++){
        f = i%50;
        if (i == 50){
	  mi.setMotionData(50, 50.0, 0.0,0.0,80,0.0);
        }
        if (i >= 50 && i<= 80){
	  mi.getMotionData(i, x, v, a);
        }
        std::cout << f << " " << x << " " << f+x << " " << (1.0+v) << " " << a << std::endl;
    }
}
