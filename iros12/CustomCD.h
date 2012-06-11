#include <hrpPlanner/CollisionDetector.h>
#include <hrpModel/Body.h>
#include "CdShape.h"

class SphereTree;

class CustomCD : public PathEngine::CollisionDetector
{
public:
    CustomCD(hrp::BodyPtr i_robot, 
             const char *i_shapefile, const char *i_pairfile,
             hrp::BodyPtr i_env,
             const char *i_envpcfile);
    ~CustomCD();
    void updatePositions();
    bool checkCollision();
private:
    std::vector<CdShape> m_shapes;
    std::vector<std::pair<const CdShape *, const CdShape *> > m_pairs;
    SphereTree *m_stree;
};
