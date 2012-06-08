#include <hrpUtil/MatrixSolvers.h>
#include <hrpUtil/Tvmet3d.h>

using namespace hrp;

/**
   @brief check if a point in a triangle or not
   @param v vertices of the triangle
   @param inside inside direction vectors
   @param p the point
   @return true if the point is in the triangle, false otherwise
 */
bool is_in_triangle(const Vector3 *v, const Vector3 *inside, const Vector3& p)
{
    for (int i=0; i<3; i++){
        Vector3 dv(p - v[i]); 
        if (dot(dv, inside[i]) < 0) return false;
    }
    return true;
}

/**
   @brief find separation distance between a point and a triangle
   @param p vertices of the triangle
   @param inside inside direction vectors
   @param n normal vector of the triangle
   @param v the point
   @param dir direction along which the point can move
   @return separation distance
 */
double find_separation_distance(const Vector3 *p, 
                                const Vector3 *inside,
                                const Vector3& n,
                                const Vector3 &v,
                                const Vector3 &dir)
{
    double ndotdir = dot(n, dir);
    //std::cout << "ndotdir:" << ndotdir << std::endl; 
    // ndotdir=0 means the line and the triangle are parallel
    if (ndotdir){
        double d = dot(v-p[0], n)/ndotdir;
        if (d < 0){
            Vector3 x(v - d*dir);
            //std::cout << "x:" << x << std::endl;
            if (is_in_triangle(p, inside, x)){
                //std::cout << "vertex:" << x << std::endl;
                //std::cout << "vertex:" << v << std::endl;
                return -d;
            }
        }
    }
    return 0;
}

/**
   @brief find separation distance between line segments
   @param v1 start point of the first line segment
   @param d1 vector from the start to the end of the first line segment
   @param v2 start point of the first line segment
   @param d2 vector from the start to the end of the second line segment
   @param dir direction along which the second line segment can move
   @return distance required to separate line segments
 */
double find_separation_distance(const Vector3 &v1, const Vector3 &d1,
                                const Vector3 &v2, const Vector3 &d2,
                                const Vector3 &dir,
                                Vector3& p1, Vector3& p2)
{
    dmatrix A(3,3);
    A(0,0) = -d1[0]; A(0,1) = d2[0]; A(0,2) = -dir[0]; 
    A(1,0) = -d1[1]; A(1,1) = d2[1]; A(1,2) = -dir[1]; 
    A(2,0) = -d1[2]; A(2,1) = d2[2]; A(2,2) = -dir[2]; 
    dvector x(3), b(3);
    for (int i=0; i<3; i++) b(i) = v1[i] - v2[i]; 
    int ret = solveLinearEquationLU(A, b, x);
#if 0
    std::cout << "A:" << A << std::endl;
    std::cout << "b:" << b << std::endl;
    std::cout << "x:" << x << std::endl;
    std::cout << "ret:" << ret << std::endl;
#endif
    if (!ret && x(0) > 0 && x(0) < 1 && x(1) > 0 && x(1) < 1 && x(2) < 0){
        p1 = v1 + x(0)*d1;
        p2 = v2 + x(1)*d2;
        //std::cout << "vertex:" << Vector3(v1+x(0)*d1) << std::endl;
        //std::cout << "vertex:" << Vector3(v2+x(1)*d2) << std::endl;
        return -x(2);
    }
    return 0;
}

/**
   @brief find separation distance between triangles
   @param v1 vertices of the first triangle
   @param v2 vertices of the second triangle
   @param dir direction along which the second triangle can move
   @return distance required to separate triangles
 */
void find_separation_distance(const Vector3 *v1, 
                              const Vector3 *v2, 
                              const Vector3& dir)
{
    double d;

    // edges
    Vector3 e1[3], e2[3];
    for (int i=0; i<3; i++){
        e1[i] = v1[(i+1)%3] - v1[i];
        e2[i] = v2[(i+1)%3] - v2[i];
    }

    // normals(normalized)
    Vector3 n1, n2;
    n1 = cross(e1[0], e1[1]);
    alias(n1) = normalize(n1); 
    n2 = cross(e2[0], e2[1]);
    alias(n2) = normalize(n2); 

    // inside directions(not normalized)
    Vector3 inside1[3], inside2[3];
    for (int i=0; i<3; i++){
        inside1[i] = cross(n1, e1[i]);
        inside2[i] = cross(n2, e2[i]);
    }

#if 0
    std::cout << "n1:" << n1 << std::endl;
    std::cout << "n2:" << n2 << std::endl;
    for (int i=0; i<3; i++){
        std::cout << "inside1[" << i << "]:" << inside1[i] << std::endl;
    }
    for (int i=0; i<3; i++){
        std::cout << "inside2[" << i << "]:" << inside2[i] << std::endl;
    }
#endif

    // project vertices of the second triangle onto the first triangle
    std::cout << "vertex check 1" << std::endl;
    for (int i=0; i<3; i++){
        d = find_separation_distance(v1, inside1, n1, v2[i], dir);
        //std::cout << d << std::endl;
        if (d){
            std::cout << Vector3(v2[i]+d*dir) << "," << v2[i] << std::endl;
        }
    }
    // project vertices of the first triangle onto the second triangle
    std::cout << "vertex check 2" << std::endl;
    Vector3 rdir(-dir);
    for (int i=0; i<3; i++){
        d = find_separation_distance(v2, inside2, n2, v1[i], rdir);
        //std::cout << d << std::endl;
        if (d){
            std::cout << v1[i] << "," << Vector3(v1[i]+d*rdir) << std::endl;
        }
    }
    std::cout << "edge check" << std::endl;
    Vector3 p1, p2;
    for (int i=0; i<3; i++){
        for (int j=0; j<3; j++){
            d = find_separation_distance(v1[i], e1[i],
                                         v2[j], e2[j],
                                         dir,
                                         p1, p2);
            //std::cout << d << std::endl;
            if (d){
                std::cout << p1 << "," << p2 << std::endl;
            }
        }
    }
}

int main()
{
    Vector3 u[] = {Vector3(-0.2,-0.2,0),
                   Vector3( 0.8,-0.2,0),
                   Vector3(-0.2, 0.8,0)};
    Vector3 v[] = {Vector3(0.0,0.0,0.1),
                   Vector3(0.0,1.0,0.1),
                   Vector3(1.0,0.0,0.1)};
    Vector3 dir(0,0,-1);
    find_separation_distance(u,v,dir);
    return 0;
}

