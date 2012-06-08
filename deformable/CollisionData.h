#ifndef __COLLISION_DATA_H__
#define __COLLISION_DATA_H__
#include <hrpUtil/Tvmet3d.h>

class CollisionData
{
public:
    CollisionData(const hrp::Vector3& i_p, const hrp::Vector3& i_n, 
                  double i_d, int i_id1, int i_id2)
        : position(i_p), normal(i_n), idepth(i_d), id1(i_id1), id2(i_id2){}
    hrp::Vector3 position;
    hrp::Vector3 normal;
    double idepth;
    int id1, id2; ///< index of primitive
};

typedef std::vector<CollisionData> CollisionDataSequence;
#endif
