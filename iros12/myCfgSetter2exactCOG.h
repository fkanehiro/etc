/*
  ランダムサンプリングした、体幹の高さ、回転、腕の関節角度から全関節角度を
　決定する
 */
#include <Model/HumanoidBodyUtil.h>
#include <Solver/TransformationConstraint.h>
#include <Solver/ComConstraint.h>
#include <Solver/ConfigurationConstraint.h>
#include <Solver/QPsolver.h>
#include <Solver/PrioritizedTaskSetSolver.h>
#include <Solver/ConstraintManager.h>
#include <Util/QPsolverUtil.h>
#include <hrpPlanner/PathPlanner.h>
#include <hrpModel/Link.h>

#define PRIO_CA   0
#define PRIO_COM  1
#define PRIO_FOOT 1
#define PRIO_HAND 2
#define PRIO_CFG  2

class myCfgSetter2
{
public:
    myCfgSetter2(motion_generator::HumanoidBodyPtr i_body) : m_body(i_body){
        for (int i=0; i<2; i++){
            m_hand[i] 
                = new TransformationConstraint(NULL, m_body->wristLink[i]);
            m_foot[i] 
                = new TransformationConstraint(NULL, m_body->ankleLink[i]);
        }
        // com constraint
        std::vector<bool> commask(3, true);
        commask[2] = false; 
        m_com = new ComConstraint(m_body, NULL, commask);
        m_cfg = new ConfigurationConstraint(m_body);
        m_tss = new PrioritizedTaskSetSolver(m_body->numJoints()+6);
        m_tss->setLambda(1e-3);
        m_cm = new ConstraintManager(m_tss, m_body, NULL);

        hrp::ivector usedJoints;
        usedJoints.resize(m_body->numJoints()+6);
        usedJoints.setZero();
        for (int i=0;i<m_body->numJoints()+6; i++){
            usedJoints[i] = 1;
        }
        //usedJoints[12] = usedJoints[13] = 0.0; // waist
        usedJoints[22] = usedJoints[29] = 0.0; // fingers
        m_cm->constrainedJoints(usedJoints);
        m_tss->addEqTask(PRIO_HAND, m_hand[0]);
        m_tss->addEqTask(PRIO_HAND, m_hand[1]);
        m_tss->addEqTask(PRIO_FOOT, m_foot[0]);
        m_tss->addEqTask(PRIO_FOOT, m_foot[1]);
        m_tss->addEqTask(PRIO_COM, m_com);
        m_tss->addEqTask(PRIO_CFG, m_cfg);
        m_initP = m_body->rootLink()->p;
        m_body->getPosture(m_initQ);
    }
    bool set(PathEngine::PathPlanner *i_planner,
             const PathEngine::Configuration &i_cfg){
        m_body->rootLink()->p[2] = i_cfg[0];
        m_body->rootLink()->R = hrp::rotFromRpy(i_cfg[1], i_cfg[2], i_cfg[3]);
        m_body->setPosture(m_initQ);
        m_body->calcForwardKinematics();
        return true;
    }
    void profile(){
    }
private:
    TransformationConstraint *m_hand[2], *m_foot[2];
    ComConstraint *m_com;
    ConfigurationConstraint *m_cfg;
    ConstraintManager *m_cm;
    PrioritizedTaskSetSolver *m_tss;
    motion_generator::HumanoidBodyPtr m_body;
    hrp::dvector m_initQ;
    hrp::Vector3 m_initP;
};
