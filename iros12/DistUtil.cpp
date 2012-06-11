#include "DistUtil.h"

double pointLineSegmentDistance(const hrp::Vector3 &p,
                                const hrp::Vector3 &p1,
                                const hrp::Vector3 &p2)
{
    hrp::Vector3 n(p1 - p2);
    double len = n.norm();
    n /= len;
    hrp::Vector3 v2(p - p2);
    double v = n.dot(v2);
    if (v < 0){
        return v2.norm();
    }else if (v > len){
        return (p1 - p).norm();
    }else{
        double l2 = v2.norm();
        return sqrt(l2*l2-v*v);
    }
}

double SegSegDist(const hrp::Vector3& u0, const hrp::Vector3& u1,
                  const hrp::Vector3& v0, const hrp::Vector3& v1)
{
    hrp::Vector3    u(u1 - u0);
    hrp::Vector3    v(v1 - v0);
    hrp::Vector3    w(u0 - v0);
    double    a = u.dot(u);        // always >= 0
    double    b = u.dot(v);
    double    c = v.dot(v);        // always >= 0
    double    d = u.dot(w);
    double    e = v.dot(w);
    double    D = a*c - b*b;       // always >= 0
    double    sc, sN, sD = D;      // sc = sN / sD, default sD = D >= 0
    double    tc, tN, tD = D;      // tc = tN / tD, default tD = D >= 0

    // compute the line parameters of the two closest points
#define EPS 1e-8
    if (D < EPS) { // the lines are almost parallel
        sN = 0.0;        // force using point P0 on segment S1
        sD = 1.0;        // to prevent possible division by 0.0 later
        tN = e;
        tD = c;
    }
    else {                // get the closest points on the infinite lines
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if (sN < 0.0) {       // sc < 0 => the s=0 edge is visible
            sN = 0.0;
            tN = e;
            tD = c;
        }
        else if (sN > sD) {  // sc > 1 => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0) {           // tc < 0 => the t=0 edge is visible
        tN = 0.0;
        // recompute sc for this edge
        if (-d < 0.0)
            sN = 0.0;
        else if (-d > a)
            sN = sD;
        else {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD) {      // tc > 1 => the t=1 edge is visible
        tN = tD;
        // recompute sc for this edge
        if ((-d + b) < 0.0)
            sN = 0;
        else if ((-d + b) > a)
            sN = sD;
        else {
            sN = (-d + b);
            sD = a;
        }
    }
    // finally do the division to get sc and tc
    sc = (fabsf(sN) < EPS ? 0.0f : sN / sD);
    tc = (fabsf(tN) < EPS ? 0.0f : tN / tD);

    hrp::Vector3 cp0(u0 + sc * u);
    hrp::Vector3 cp1(v0 + tc * v);

    // get the difference of the two closest points
    hrp::Vector3 dP(cp0 - cp1); 

    return dP.norm();   // return the closest distance
}


