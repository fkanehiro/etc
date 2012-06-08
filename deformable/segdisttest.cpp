#include <hrpUtil/Tvmet3d.h>
#include <iostream>

using namespace hrp;

inline double SegSegDist(const Vector3& u0, const Vector3& u,
			 const Vector3& v0, const Vector3& v,
			 Vector3& cp0, Vector3& cp1)
{
    Vector3   w(u0 - v0);
    double    a = dot(u,u);        // always >= 0
    double    b = dot(u,v);
    double    c = dot(v,v);        // always >= 0
    double    d = dot(u,w);
    double    e = dot(v,w);
    double    D = a*c - b*b;       // always >= 0
    double    sc, sN, sD = D;      // sc = sN / sD, default sD = D >= 0
    double    tc, tN, tD = D;      // tc = tN / tD, default tD = D >= 0

    // compute the line parameters of the two closest points
#define EPS 1e-16
    std::cout << "D:" << D << std::endl;
    if (D < EPS) { // the lines are almost parallel
        sN = 0.0;        // force using point P0 on segment S1
        sD = 1.0;        // to prevent possible division by 0.0 later
        tN = e;
        tD = c;
	std::cout << "case1" << std::endl;
    }
    else {                // get the closest points on the infinite lines
        sN = (b*e - c*d);
        tN = (a*e - b*d);
	std::cout << "case2" << std::endl;
        if (sN < 0.0) {       // sc < 0 => the s=0 edge is visible
            sN = 0.0;
            tN = e;
            tD = c;
	    std::cout << "case2.1" << std::endl;
        }
        else if (sN > sD) {  // sc > 1 => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
	    std::cout << "case2.2" << std::endl;
        }
    }

    if (tN < 0.0) {           // tc < 0 => the t=0 edge is visible
        tN = 0.0;
        // recompute sc for this edge
	std::cout << "case3" << std::endl;
        if (-d < 0.0){
	  sN = 0.0;
	  std::cout << "case3.1" << std::endl;
	}else if (-d > a){
            sN = sD;
	    std::cout << "case3.2" << std::endl;
	}else {
            sN = -d;
            sD = a;
	    std::cout << "case3.3" << std::endl;
        }
    }
    else if (tN > tD) {      // tc > 1 => the t=1 edge is visible
        tN = tD;
        // recompute sc for this edge
	std::cout << "case4" << std::endl;
        if ((-d + b) < 0.0){
            sN = 0;
	    std::cout << "case4.1" << std::endl;
	}else if ((-d + b) > a){
            sN = sD;
	    std::cout << "case4.2" << std::endl;
	}else {
            sN = (-d + b);
            sD = a;
	    std::cout << "case4.3" << std::endl;
        }
    }
    // finally do the division to get sc and tc
    std::cout << "sN:" << sN << ", tN:" << tN << std::endl;
    std::cout << "sD:" << sD << ", tD:" << tD << std::endl;
    sc = (fabsf(sN) < EPS ? 0.0f : sN / sD);
    tc = (fabsf(tN) < EPS ? 0.0f : tN / tD);

    cp0 = u0 + sc * u;
    cp1 = v0 + tc * v;

    // get the difference of the two closest points
    Vector3 dP(cp0 - cp1); 
    std::cout << "dP|u = " << dot(dP,u) << std::endl;
    std::cout << "dP|v = " << dot(dP,v) << std::endl;

    return norm2(dP);   // return the closest distance
}

int main()
{
  Vector3 v0(-1.19999492,-2.02842712,-0.00010001);
  Vector3 d0( 2.82842207, 2.82843256, 0.00000000);
  Vector3 v1( 1.00000000, 0.00000000, 0.00000000);
  Vector3 d1(-1.00000000, 0.00000000, 0.00000000);

  Vector3 p0, p1;
  double d = SegSegDist(v0,d0,v1,d1,p0,p1);
  Vector3 n(p0-p1);
  n /= norm2(n);
  
  printf("d:%11.8f\n", d);
  printf("p0:(%11.8f %11.8f %11.8f)\n", p0[0], p0[1], p0[2]);
  printf("p1:(%11.8f %11.8f %11.8f)\n", p1[0], p1[1], p1[2]);
  printf("n:(%11.8f %11.8f %11.8f)\n", n[0], n[1], n[2]);
  return 0;
}
