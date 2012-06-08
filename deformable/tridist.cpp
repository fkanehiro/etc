#include <iostream>
#include "hrpCollision/Opcode/Opcode.h"

/**
 * @brief compute the minimum distance and the closest points between two line segments
 * 
 * This function is implemented referring the following webpage
 * http://www.softsurfer.com/Archive/algorithm_0106/algorithm_0106.htm
 * @param u0 start point of the first line segment
 * @param u vector from u0 to the other end point of the first line segment
 * @param v0 start point of the second line segment
 * @param v vector from v0 the other end point of the second line segment
 * @param cp0 the closest point on the first line segment 
 * @param cp1 the closest point on the second line segment
 * @return the minimum distance
 */
inline float SegSegDist(const Point& u0, const Point& u,
                        const Point& v0, const Point& v,
                        Point& cp0, Point& cp1)
{
    Point    w = u0 - v0;
    float    a = u|u;        // always >= 0
    float    b = u|v;
    float    c = v|v;        // always >= 0
    float    d = u|w;
    float    e = v|w;
    float    D = a*c - b*b;       // always >= 0
    float    sc, sN, sD = D;      // sc = sN / sD, default sD = D >= 0
    float    tc, tN, tD = D;      // tc = tN / tD, default tD = D >= 0

    // compute the line parameters of the two closest points
#define EPS 1e-16
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

    cp0 = u0 + sc * u;
    cp1 = v0 + tc * v;

    // get the difference of the two closest points
    Point dP = cp0 - cp1; 

    return dP.Magnitude();   // return the closest distance
}

/**
 * @brief compute signed distance between a point and a plane
 * @param P a point
 * @param pointOnPolane a point on the plane
 * @param n normal vector of the plane
 * @param cp the closest point on the plane from P
 */
inline float PointPlaneDist(const Point& P, const Point& pointOnPlane, const Point& n,
                     Point& cp)
{
    Point v = P - pointOnPlane;
    float l = v|n;
    cp = P-l*n;
    return l;
}

// see DistFuncs.h
float PointSegDist(const Point& P, const Point& u0, const Point& u1)
{
    Point v = P - u0;
    Point dir = u1 - u0;
    float l = dir.Magnitude();
    dir /= l;
    
    float x = v|dir;
    if (x < 0){
        return v.Magnitude();
    }else if (x > l){
        Point dv = P - u1;
        return dv.Magnitude();
    }else{
        return sqrtf(v.SquareMagnitude() - x*x);
    }
}

/**
 * @brief check whether a point is in Voroni region of a face
 * @param p a point to be tested
 * @param vertices vertices of the triangle
 * @param edges edges of the triangle
 * @param n normal vector of the triangle
 * @return true if the point is in the Voronoi region, false otherwise
 */
inline bool PointFaceAppTest(const Point& P, const Point** vertices,
                             const Point* edges, const Point& n)
{
    Point v, outer;
    for (unsigned int i=0; i<3; i++){
        v = P - *vertices[i];
        outer = edges[i]^n;
        if ((v|outer)>0) return false;
    }
    return true;
}

// see DistFuncs.h
float TriTriDist(const Point& U0, const Point& U1, const Point& U2,
                 const Point& V0, const Point& V1, const Point& V2,
                 Point& cp0, Point& cp1)
{
    const Point* uvertices[] = {&U0, &U1, &U2};
    const Point* vvertices[] = {&V0, &V1, &V2};
    float min_d, d;
    Point p0, p1, n, vec;
    bool overlap = true;

    // edges
    Point uedges[3], vedges[3];
    for (unsigned int i=0; i<3; i++){
        uedges[i] = *uvertices[(i+1)%3] - *uvertices[i];
        vedges[i] = *vvertices[(i+1)%3] - *vvertices[i];
    }

    // set initial values
    cp0 = U0;
    cp1 = V0;
    min_d = (cp0-V0).Magnitude();

    // vertex-vertex, vertex-edge, edge-edge
    for (unsigned int i=0; i<3; i++){
        for (unsigned int j=0; j<3; j++){
            d = SegSegDist(*uvertices[i], uedges[i],
                           *vvertices[j], vedges[j],
                           p0, p1);
            n = p0 - p1;
            if (d <= min_d){
                min_d = d;
                cp0 = p0;
                cp1 = p1;
                // check overlap
                vec = *uvertices[(i+2)%3] - cp0;
                float u = (vec|n)/min_d;
                vec = *vvertices[(j+2)%3] - cp1;
                float v = (vec|n)/min_d;
                // n directs from v -> u
                if (u>=0 && v<=0) return min_d;

                if (u > 0) u = 0;
                if (v < 0) v = 0;
                if ((n.Magnitude() + u - v) > 0){
                    overlap = false;
                }
            }
        }
    }

    Point un = uedges[0]^uedges[1];
    un.Normalize();
    Point vn = vedges[0]^vedges[1];
    vn.Normalize();

    // vertex-face, edge-face, face-face
    // plane-triangle overlap test
    float ds[3];
    int rank;
    //   u and plane including v
    rank = -1;
    for (int i=0; i<3; i++){
        vec = *uvertices[i] - *vvertices[0];
        ds[i] = vec|vn; 
    }
    if (ds[0] > 0 && ds[1] > 0 && ds[2] > 0){
        // find rank of the nearest vertex to the plane
        rank = ds[0] > ds[1] ? 1 : 0;
        rank = ds[rank] > ds[2] ? 2 : rank;
    }else if(ds[0] < 0 && ds[1] < 0 && ds[2] < 0){
        // find rank of the nearest vertex to the plane
        rank = ds[0] > ds[1] ? 0 : 1;
        rank = ds[rank] > ds[2] ? rank : 2;
    }
    if (rank >=0){
        overlap = false;
        if (PointFaceAppTest(*uvertices[rank], vvertices, vedges, vn)){
            min_d = fabsf(PointPlaneDist(*uvertices[rank], *vvertices[0], vn, p1));
            cp0 = *uvertices[rank];
            cp1 = p1;
            return min_d;
        }
    }

    //   v and plane including u
    rank = -1;
    for (int i=0; i<3; i++){
        vec = *vvertices[i] - *uvertices[0];
        ds[i] = vec|un; 
    }
    if (ds[0] > 0 && ds[1] > 0 && ds[2] > 0){
        // find rank of the nearest vertex to the plane
        rank = ds[0] > ds[1] ? 1 : 0;
        rank = ds[rank] > ds[2] ? 2 : rank;
    }else if(ds[0] < 0 && ds[1] < 0 && ds[2] < 0){
        // find rank of the nearest vertex to the plane
        rank = ds[0] > ds[1] ? 0 : 1;
        rank = ds[rank] > ds[2] ? rank : 2;
    }
    if (rank >= 0){
        overlap = false;
        if (PointFaceAppTest(*vvertices[rank], uvertices, uedges, un)){
            min_d = fabsf(PointPlaneDist(*vvertices[rank], *uvertices[0], un, p0));
            cp0 = p0;
            cp1 = *vvertices[rank];
            return min_d;
        }
    }

    if (overlap){
        return 0;
    }else{
        return min_d;
    }
}

// see DistFuncs.h
float SegSegDist(const Point& u0, const Point& u1,
                 const Point& v0, const Point& v1)
{
    Point cp0, cp1;
    return SegSegDist(u0, u1-u0, v0, v1-v0, cp0, cp1);
}

int main(int argc, char *argv[])
{
    Point u0(0,0,0), u1(1,0,0), u2(0,1,0);
    Point v0(-0.1,0.1,-0.1), v1(0.1,0.1,-0.1), v2(0,0.1,0.1);
    Point cp0, cp1;

    double d = TriTriDist(u0,u1,u2,v0,v1,v2,cp0,cp1);
    std::cout << "d=" << d << std::endl;
    return 0;
}

