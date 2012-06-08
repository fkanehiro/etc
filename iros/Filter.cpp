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
#include "Filter.h"

using namespace motion_generator;
using namespace motion_interpolator;
using namespace hrp;

Filter::Filter(HumanoidBodyPtr i_body, int i_arm, bool i_balance) : 
    m_body(i_body), m_arm(i_arm)
{
    m_refBody = new HumanoidBody(*m_body);
    Link *hand = m_body->wristLink[i_arm];

    // constraint for right hand
    m_hand = new TransformationConstraint(NULL, hand);

    // transformation constraint for feet
    m_foot[0] = new TransformationConstraint(NULL, m_body->ankleLink[RIGHT]);
    m_foot[1] = new TransformationConstraint(NULL, m_body->ankleLink[LEFT]);

    // com constraint
    std::vector<bool> commask(3, true);
    commask[2] = false; 
    m_com = new ComConstraint(m_body, NULL, commask);

    m_cfg = new ConfigurationConstraint(m_body);

    m_tss = new PrioritizedTaskSetSolver(m_body->numJoints()+6);
    m_tss->setLambda(1e-1);

    m_cm = new ConstraintManager(m_tss, m_body, NULL);

    dvector usedJoints;
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
    usedJoints[22] = usedJoints[29] = 0.0; // fingers
#endif

    m_cm->constrainedJoints(usedJoints);
    //cm.enableJointLimits(false);

    m_tss->addEqTask(0, m_foot[0]);
    m_tss->addEqTask(0, m_foot[1]);
    if (i_balance) m_tss->addEqTask(0, m_com);
    // 1 is for distance constraints
    m_tss->addEqTask(2, m_hand);
    m_tss->addEqTask(3, m_cfg);
}

void Filter::setupDistanceConstraints()
{
    // clear old tasks
    m_tss->clearTasks(1);
    
    // set active tasks
    for (unsigned int i=0; i<m_dists.size(); i++){
        m_dists[i]->computeDistance();
        if (m_dists[i]->isActive()){
            m_dists[i]->update();
            m_tss->addIneqTask(1, m_dists[i]);
        }
    }
    for (unsigned int i=0; i<m_selfCAconsts.size(); i++){
        m_selfCAconsts[i]->computeDistance();
        if (m_selfCAconsts[i]->isActive()){
            m_selfCAconsts[i]->update();
            m_tss->addIneqTask(1, m_selfCAconsts[i]);
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

void Filter::filter(const dvector& q, const Vector3 &p, const Matrix33 &R)
{
    m_refBody->setPosture(q, p, R);
    m_refBody->calcForwardKinematics();
    
    // update reference
    Vector3 currentP;
    m_body->calcGraspPosition(m_arm, currentP);
    Vector3 refP;
    m_refBody->calcGraspPosition(m_arm, refP);
    Vector3 dv(refP - currentP);
    double l = dv.norm();
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
    m_hand->referenceTransformation(refP, refR);

    setupDistanceConstraints();

    for (int i=0; i<2; i++) m_foot[i]->update();
    m_hand->update();
    m_com->update();

#if 1
    const double maxdq = 0.01;
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
    
    dvector dq(m_body->numJoints() + 6); 
    for (int i=0; i<m_body->numJoints(); i++){
        dq[i] = q[i] - m_body->joint(i)->q;
    }
    for (int i=0; i<6; i++){
        dq[m_body->numJoints()+i] = 0;
    }
    int ret = m_cm->solve(dq);
    if (ret){
        std::cerr << "information from QPsolver = " << ret << std::endl;
        getchar();
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
