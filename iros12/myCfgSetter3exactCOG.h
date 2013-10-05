/*
  ランダムサンプリングした、体幹の高さ、回転、腕の関節角度から全関節角度を
　決定する
 */
#include <Math/MathFunction.h>
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

#define PRIO_TRUNK 0
#define PRIO_COM   0
#define PRIO_FOOT  0
#define PRIO_HAND  0
#define PRIO_CFG   2

class myCfgSetter3
{
public:
  myCfgSetter3(motion_generator::HumanoidBodyPtr i_body,
               const hrp::Vector3 &i_goalP) 
      : m_body(i_body), m_goalP(i_goalP){
      std::vector<bool> trkmask(6, true);
      trkmask[0] = trkmask[1] = false;
      m_trunk
          = new TransformationConstraint(NULL, m_body->rootLink(), trkmask);
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
      //m_cm->enableJointLimits(false);
      m_cm->dt(0.1);
      
      m_tss->addEqTask(PRIO_TRUNK, m_trunk);
      m_tss->addEqTask(PRIO_HAND, m_hand[0]);
      //m_tss->addEqTask(PRIO_HAND, m_hand[1]);
      m_tss->addEqTask(PRIO_FOOT, m_foot[0]);
      m_tss->addEqTask(PRIO_FOOT, m_foot[1]);
      m_tss->addEqTask(PRIO_COM, m_com);
      //m_tss->addEqTask(PRIO_CFG, m_cfg);
      m_initP = m_body->rootLink()->p;
      m_initR = m_body->rootLink()->R;
      m_body->getPosture(m_initQ);
  }
    bool set(PathEngine::PathPlanner *i_planner,
             const PathEngine::Configuration &i_cfg){
        m_body->rootLink()->p = m_initP;
        m_body->rootLink()->R = m_initR;
        m_body->setPosture(m_initQ);
        m_body->calcForwardKinematics();
        
        hrp::Vector3 p = m_initP;
        p[2] = i_cfg[0];
        m_trunk->referenceTransformation(
            p, 
            hrp::rotFromRpy(i_cfg[1], i_cfg[2], i_cfg[3])
            );
        hrp::Matrix33 bulbR = hrp::rotFromPitch(-M_PI/2);
        hrp::Matrix33 R = hrp::rotFromRpy(i_cfg[4], i_cfg[5], i_cfg[6]); 
        //hrp::Matrix33 R = hrp::rotFromRpy(0,0,M_PI/2);
        hrp::Matrix33 wristR(bulbR*R);
        // assuming getGraspLocalPosition(0) and (1) are same
        hrp::Vector3 wristP(m_goalP - wristR*m_body->getGraspLocalPosition(0));
        m_hand[0]->referenceTransformation(wristP, wristR);

        hrp::dvector dq(m_body->numJoints()+6);
        for (int j=0; j<10; j++){
            m_trunk->update();
            for (int i=0; i<2; i++){
                m_foot[i]->update();
                m_hand[i]->update();
            }
            m_com->update();
            m_cfg->update();
            
            dq.setZero();
            int ret = m_cm->solve(dq);
            if (ret){
                std::cerr << "information from QPsolver = " << ret << std::endl;
            }
            m_body->calcForwardKinematics();
            if (m_foot[0]->error(dq).norm()+
                m_foot[1]->error(dq).norm()+
                m_com->error(dq).norm()+
                m_hand[0]->error(dq).norm() < 1e-3) return true;
        }
        return false;
    }
    void profile(){
    }
private:
    TransformationConstraint *m_hand[2], *m_foot[2], *m_trunk;
    ComConstraint *m_com;
    ConfigurationConstraint *m_cfg;
    ConstraintManager *m_cm;
    PrioritizedTaskSetSolver *m_tss;
    motion_generator::HumanoidBodyPtr m_body;
    hrp::dvector m_initQ;
    hrp::Vector3 m_initP, m_goalP;
    hrp::Matrix33 m_initR;
};
