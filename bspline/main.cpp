#include <fstream>
#include "bspline.h"

using namespace hrp;

int main(int argc, char *argv[])
{
    dvector t(100);
    for (unsigned int i=0; i<100; i++) t[i] = i*0.1;

    dmatrix splines;

    createSplineCurves(3,3,t,splines);

    std::ofstream ofs("spline.txt");
    for (unsigned int i=0; i<splines.size1(); i++){
	for (unsigned int j=0; j<splines.size2(); j++){
	    ofs << splines(i,j) << " "; 
	}
	ofs << std::endl;
    }
}

