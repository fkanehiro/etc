#include <Model/HumanoidBody.h>

class TransformationConstraint;
class ComConstraint;
class ConfigurationConstraint;
class DistanceConstraint;
class Sphere;
class DistancePair;
class ConstraintManager;
class PrioritizedTaskSetSolver;
class DistanceGeometry;

class Filter
{
public:
    Filter(motion_generator::HumanoidBodyPtr i_body);
    ~Filter();
    void init(const hrp::dvector&q, const hrp::Vector3& p, const hrp::Matrix33& R);
    bool filter(const hrp::dvector& q, const hrp::Vector3& p, const hrp::Matrix33 &R);
    void setDistanceConstraints(const std::vector<DistanceConstraint *>& i_dists);
    void addSelfCollisionAvoidanceConstraint(DistanceConstraint *i_dc);
    void setRobotShapes(const std::vector<DistanceGeometry *> i_dg);
    void considerBalance(bool flag);
    void selectArm(int i);
    double error();
    void duration(double i_tm) { m_duration = i_tm; }
private:
    void setupDistanceConstraints();

    TransformationConstraint *m_hand[2], *m_foot[2];
    ComConstraint *m_com;
    ConfigurationConstraint *m_cfg;
    std::vector<DistanceConstraint *> m_dists;
    std::vector<DistanceConstraint *> m_selfCAconsts;
    std::vector<DistanceGeometry *> m_shapes;
    std::vector<Sphere *> m_spheres;
    std::vector<DistancePair *> m_pairs;
    ConstraintManager *m_cm;
    PrioritizedTaskSetSolver *m_tss;
    motion_generator::HumanoidBodyPtr m_body, m_refBody;
    int m_arm;
    hrp::dvector m_dq;
    double m_time, m_duration;
};

