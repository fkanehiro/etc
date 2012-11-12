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
    CustomCD(hrp::BodyPtr i_robot, 
             const char *i_shapefile, const char *i_pairfile,
             hrp::BodyPtr i_env,
             SphereTree *i_stree);
    ~CustomCD();
    bool loadPointCloud(hrp::BodyPtr i_env, const char *i_envpcfile);
    void updatePositions();
    bool checkCollision();
    std::vector<CdShape>& shapes() { return m_shapes; }
    void tolerance(double i_d) { m_tolerance = i_d; }
    double tolerance() const { return m_tolerance; }
private:
    std::vector<CdShape> m_shapes; ///< shapes of the robot
    /// self-collision check pairs
    std::vector<std::pair<const CdShape *, const CdShape *> > m_pairs;
    SphereTree *m_stree; ///< sphere tree of the environment 
    bool m_outerTree;
    double m_tolerance;
};
