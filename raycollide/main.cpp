#include <hrpModel/ModelLoaderUtil.h>
#include <hrpCollision/Opcode/Opcode.h>
#include <hrpModel/Link.h>
#include <hrpCollision/ColdetModelSharedDataSet.h>
#include <iostream>

using namespace hrp;
using namespace OpenHRP;

int main(int argc, char *argv[])
{
    BodyInfo_var binfo = loadBodyInfo("file:///home/kanehiro/spiralStair.wrl", argc, argv);
    BodyPtr body(new Body());
    loadBodyFromBodyInfo(body, binfo, true);

    Opcode::RayCollider RC;
#define DZ 1.0
    Opcode::CollisionFace CF;
    Opcode::SetupClosestHit(RC, CF);
    udword Cache;
    Link *l = body->rootLink();
    const Matrix33 R = l->attitude();
    const Vector3& p = l->p;
    Matrix4x4 transform;
    transform.Set((float)R(0,0), (float)R(1,0), (float)R(2,0), 0.0f,
                  (float)R(0,1), (float)R(1,1), (float)R(2,1), 0.0f,
                  (float)R(0,2), (float)R(1,2), (float)R(2,2), 0.0f,
                  (float)p(0),   (float)p(1),   (float)p(2),   1.0f);

    double fx=-1.71412, fy=0.506532, fth=-4.7007;
    double polygon[4][2] = {{0.13, 0.075},
                            {-0.10, 0.075},
                            {-0.10, -0.055},
                            {0.13, -0.055}};
    double costh = cos(fth);
    double sinth = sin(fth);
    std::cout << fx << " " << fy << " 0" << std::endl;
    for (unsigned int i=0; i<4; i++){
        double x = fx + costh*polygon[i][0] - sinth*polygon[i][1];
        double y = fy + sinth*polygon[i][0] + costh*polygon[i][1];
        Ray world_ray(Point(x, y, DZ),
                      Point(0,0,-1));
        Opcode::SetupClosestHit(RC, CF);
        RC.Collide(world_ray, l->coldetModel->getDataSet()->model, &transform, &Cache);
        std::cout << x << " " << y << " " << CF.mDistance << std::endl;
    }
}
