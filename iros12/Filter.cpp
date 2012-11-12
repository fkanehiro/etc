#include <iostream>
#include <fstream>
#include <Model/HumanoidBodyUtil.h>
#include <Solver/TransformationConstraint.h>
#include <Solver/ComConstraint.h>
#include <Solver/ConfigurationConstraint.h>
#include <Solver/QPsolver.h>
#include <Solver/PrioritizedTaskSetSolver.h>
#include <Solver/ConstraintManager.h>
#include <Util/QPsolverUtil.h>
#include <hrpModel/Link.h>
#include <Math/Interpolator/HoffArbib.h>
#include <Math/Physics.h>
#include <Solver/CappedCylinder.h>
#include <Solver/Sphere.h>
#include <Solver/CappedCylinderSpherePair.h>
#include <Solver/SphereSpherePair.h>
#include <Solver/DistanceConstraint.h>
#include <Solver/QPsolverQuadProg.h>
#include "Filter.h"

using namespace motion_generator;
using namespace motion_interpolator;
using namespace hrp;

#define PRIO_CA   0
#define PRIO_COM  1
#define PRIO_FOOT 1
#define PRIO_HAND 2
#define PRIO_CFG  2

Filter::Filter(HumanoidBodyPtr i_body) : 
    m_body(i_body), m_arm(-1), m_dq(m_body->numJoints()+6), m_duration(0)
{
    m_refBody = HumanoidBodyPtr(new HumanoidBody(*m_body));
    
    // constraint for right hand
    for (int i=0; i<2; i++){
        m_hand[i] = new TransformationConstraint(NULL, m_body->wristLink[i]);
        m_hand[i]->setLocalPoint(m_body->getGraspLocalPosition(i));
    }

    // transformation constraint for feet
    m_foot[0] = new TransformationConstraint(NULL, m_body->ankleLink[RIGHT]);
    m_foot[1] = new TransformationConstraint(NULL, m_body->ankleLink[LEFT]);

    // com constraint
    std::vector<bool> commask(3, true);
    commask[2] = false; 
    m_com = new ComConstraint(m_body, NULL, commask);

    m_cfg = new ConfigurationConstraint(m_body);

    m_tss = new PrioritizedTaskSetSolver(m_body->numJoints()+6);
    m_tss->setLambda(1e-3);
    QPsolverQuadProg *solver = dynamic_cast<QPsolverQuadProg *>(m_tss->solver());
    if (solver) solver->setStrictErrorCheck(true);

    m_cm = new ConstraintManager(m_tss, m_body, NULL);

    ivector usedJoints;
    usedJoints.resize(m_body->numJoints()+6);
    usedJoints.setZero();
#if 0
    // legs
    for (int i=0; i<2; i++){
        Link *l = m_body->ankleLink[i];
        while (l != m_body->rootLink()){
            usedJoints[l->jointId] = 1;
            l = l->parent;
        }
    }
    // reaching arm
    Link *l = hand;
    while (l != m_body->chestLink){
        usedJoints[l->jointId] = 1;
        l = l->parent;
    }

    usedJoints[m_body->waistPjoint->jointId] = 1; 
    usedJoints[m_body->waistYjoint->jointId] = 1; 
    usedJoints[m_body->numJoints()  ] = 1; // base X
    usedJoints[m_body->numJoints()+1] = 1; // base Y
    usedJoints[m_body->numJoints()+2] = 1; // base Z
    usedJoints[m_body->numJoints()+3] = 1; // base roll
    usedJoints[m_body->numJoints()+4] = 1; // base pitch
    usedJoints[m_body->numJoints()+5] = 1; // base yaw
#else
    for (int i=0;i<m_body->numJoints()+6; i++){
        usedJoints[i] = 1;
    }
    //usedJoints[12] = usedJoints[13] = 0.0; // waist
    usedJoints[22] = usedJoints[29] = 0.0; // fingers
#endif

    m_cm->constrainedJoints(usedJoints);
    //cm.enableJointLimits(false);

    m_tss->addEqTask(PRIO_FOOT, m_foot[0]);
    m_tss->addEqTask(PRIO_FOOT, m_foot[1]);
    m_tss->addEqTask(PRIO_COM, m_com);
    m_tss->addEqTask(PRIO_CFG, m_cfg);
    //m_tss->addEqTask(PRIO_CFG, m_cfg);
}

void Filter::setupDistanceConstraints()
{
    // clear old tasks
    m_tss->clearTasks(PRIO_CA);
    
    // set active tasks
    for (unsigned int i=0; i<m_dists.size(); i++){
        m_dists[i]->computeDistance();
        if (m_dists[i]->isActive()){
            m_dists[i]->update();
            m_tss->addIneqTask(PRIO_CA, m_dists[i]);
        }
#if 0
        if (m_dists[i]->distance() < m_dists[i]->securityDistance()-0.001){
            std::cout << "Warning: distance is smaller than Ds("
                      << m_dists[i]->distance() << ")" << std::endl;
        }
#endif
    }
    for (unsigned int i=0; i<m_selfCAconsts.size(); i++){
        m_selfCAconsts[i]->computeDistance();
        if (m_selfCAconsts[i]->isActive()){
            m_selfCAconsts[i]->update();
            m_tss->addIneqTask(PRIO_CA, m_selfCAconsts[i]);
        }
    }
    //std::cout << m_tss->taskSet()[1].nIneqTasks() << " tasks" << std::endl;
}

void Filter::setDistanceConstraints(const std::vector<DistanceConstraint *>& i_dists)
{
    m_dists = i_dists;
}

Filter::~Filter()
{
    delete m_cfg;
    delete m_com;
    for (int i=0; i<2; i++) delete m_foot[i];
    delete m_tss;
    delete m_cm;
}

void Filter::init(const dvector&q, const Vector3& p, const Matrix33& R)
{
    m_time = 0;
    m_body->setPosture(q, p, R);
    m_body->calcForwardKinematics();
    m_refBody->setPosture(q, p, R);
    m_refBody->calcForwardKinematics();

    // set references
    for (int i=0; i<2; i++){
        m_foot[i]->referenceTransformation(m_body->ankleLink[i]->p,
                                           m_body->ankleLink[i]->R);
    } 
    Vector3 com = m_body->calcCM();
    m_com->referenceCom(com);
}

bool Filter::filter(const dvector& q, const Vector3 &p, const Matrix33 &R)
{
    dvector weight(6);
    double w = 1.0;
    if (m_time > m_duration) w += (m_time - m_duration)*3;
    for (int i=0; i<3; i++){
        weight[i] = w;
        weight[i+3] = w*0.1;
    }
    m_time += DT;

    m_refBody->setPosture(q, p, R);
    m_refBody->calcForwardKinematics();
    
    // update reference
    if (m_arm==0||m_arm==1){
        Vector3 currentP;
        m_body->calcGraspPosition(m_arm, currentP);
        Vector3 refP;
        m_refBody->calcGraspPosition(m_arm, refP);
        Vector3 dv(refP - currentP);
        double l = dv.norm();
        //std::cout << m_time << ":l=" << l << ",w=" << weight[0] << std::endl;
#define PLIMIT 0.005
        if (l > PLIMIT){
            //std::cout << "p limit" << std::endl;
            refP = currentP + PLIMIT/l*dv;
        } 
        Matrix33 &currentR = m_body->wristLink[m_arm]->R;
        Matrix33 currentRt(currentR.transpose());
        Matrix33 refR(m_refBody->wristLink[m_arm]->R); 
        Matrix33 dR(currentRt*refR);
        Vector3 omega(omegaFromRot(dR));
        double w = omega.norm();
#define RLIMIT 0.005
        if (w > RLIMIT){
            //std::cout << "r limit" << std::endl;
            calcRodrigues(dR, Vector3(omega/w), RLIMIT);
            refR = currentR*dR;
        }
        m_hand[m_arm]->referenceTransformation(refP, refR);
        m_hand[m_arm]->stiffness(weight);
        m_hand[m_arm]->update();
    }

    setupDistanceConstraints();

    for (int i=0; i<2; i++) m_foot[i]->update();
    m_com->update();

#if 1
    const double maxdq = 0.002; // AR
    //const double maxdq = 0.01;
    double max = 0;
    dvector v(m_body->numJoints());
    for (int i=0; i<m_body->numJoints(); i++){
        double dq = q[i] - m_body->joint(i)->q;
        if (fabs(dq) > max) max = fabs(dq);
        v[i] = dq;
    }
    //std::cout << "max = " << max << std::endl;
    if (max > maxdq) v *= maxdq/max;
    dvector current(m_body->numJoints());
    m_body->getPosture(current);
    m_cfg->referenceConfiguration(current+v);
#else
    m_cfg->referenceConfiguration(q);
#endif
    m_cfg->update();
    
    for (int i=0; i<m_body->numJoints(); i++){
        m_dq[i] = q[i] - m_body->joint(i)->q;
    }
    for (int i=0; i<6; i++){
        m_dq[m_body->numJoints()+i] = 0;
    }
    int ret = m_cm->solve(m_dq);
    if (ret){
        std::cerr << "information from QPsolver = " << ret << std::endl;
        return false;
    }else{
        return true;
    }
}

void Filter::addSelfCollisionAvoidanceConstraint(DistanceConstraint *i_dc)
{
    m_selfCAconsts.push_back(i_dc);
}

void Filter::setRobotShapes(const std::vector<DistanceGeometry *> i_dg)
{
    m_shapes = i_dg;
}

double Filter::error()
{
    if (m_arm == 0 || m_arm == 1){
        return m_hand[m_arm]->error(m_dq).norm();
    }else{
        return 0;
    }
}

void Filter::considerBalance(bool flag)
{
    if (flag){
        m_tss->addEqTask(PRIO_COM, m_com);
    }else{
        m_tss->removeEqTask(PRIO_COM, m_com);
    }
}

void Filter::selectArm(int i)
{
    m_arm = i;
    m_tss->removeEqTask(PRIO_HAND, m_hand[0]);
    m_tss->removeEqTask(PRIO_HAND, m_hand[1]);
    if (i==0||i==1) m_tss->addEqTask(PRIO_HAND, m_hand[i]);
}
