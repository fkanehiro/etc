#include <iostream>
#include <fstream>
#include <vector>
#include "bspline.h"

using namespace hrp;

void createSplineCurves(unsigned int d, unsigned int m,
			const dvector &t, dmatrix& o_splines)
{
#if 1
    std::cout << "t:" << t << std::endl;
    std::cout << "d:" << d << std::endl;
    std::cout << "m:" << m << std::endl;
#endif
    const unsigned int dknot = t.size()/(m+d+2);
    const unsigned int nknots = t.size()/dknot;
#if 1
    std::cout << "dknot:" << dknot << std::endl;
    std::cout << "nknots:" << nknots << std::endl;
#endif
    std::vector<unsigned int> knots(nknots);
    for (unsigned int i=0; i<nknots; i++){
	knots[i] = i*dknot;
    }

#if 1
    std::cout << "knots:";
    for (unsigned int i=0; i<nknots; i++){
	std::cout << knots[i] << " ";
    }
    std::cout << std::endl;
#endif

    dmatrix bk(t.size(), nknots-1);
    for (unsigned int i=0; i<knots.size()-1; i++){
	for (unsigned int j=0; j<t.size(); j++){
	    bk(j,i)=0.0;
	}
	for (unsigned int j=0; j<knots[i+1]-knots[i]; j++){
	    bk(knots[i]+j,i) = 1;
	}
    }
    std::cout << "bk:" << bk << std::endl;

    for (unsigned int j=1; j<=d; j++){
	//std::cout << "ln:" << bk.size1() << std::endl;
        unsigned int ln = bk.size2();
	dmatrix bkkk(t.size(), ln-1);
	for (unsigned int k=0; k<ln-1; k++){
	    for (unsigned int l=0; l<t.size(); l++){
                //std::cout << j << "," << k << "," << l << std::endl;
                double k1 = (t[l] - t[knots[k]])/(t[knots[k+j]]-t[knots[k]]);
                //std::cout << "k1:" << k1 << std::endl;
                double k2 = (t[knots[k+j+1]]-t[l])/(t[knots[k+j+1]]-t[knots[k+1]]);
                //std::cout << "k2:" << k2 << std::endl;
		bkkk(l, k) = k1*bk(l, k)+k2*bk(l,k+1);
	    }
	}
	//std::cout << "bkkk:" << bkkk << std::endl;
	bk = bkkk;
    } 
    o_splines = bk;
}

