#include "cfgSetterBase.h"

class myCfgSetter : public cfgSetterBase
{
public:
    myCfgSetter(hrp::BodyPtr i_body,
                const hrp::Vector3 &i_goalP)
        : cfgSetterBase(i_body), m_goalP(i_goalP), m_q(i_body->numJoints()){
        hrp::Link *wrist[2] = {m_body->link("RARM_JOINT5"),
                               m_body->link("LARM_JOINT5")};
        hrp::Link *chest = m_body->link("CHEST_JOINT1");
        for (int i=0; i<2; i++){
            m_arm[i] = m_body->getJointPath(chest, wrist[i]);
        }
        m_trunk = m_body->getJointPath(m_body->rootLink(), chest);
        for (int i=0; i<4; i++){
            m_ikFailCount[i] = 0;
        }
        m_q.resize(m_body->numJoints());
        for (int i=0; i<m_body->numJoints(); i++){
            hrp::Link *j = m_body->joint(i);
            if (j) m_q[i] = j->q;
        }
    }
    bool set(PathEngine::PathPlanner *i_planner,
             const PathEngine::Configuration &i_cfg){
        setBase(i_cfg[0], i_cfg[1], i_cfg[2], i_cfg[3]);
        m_trunk->calcForwardKinematics();

        hrp::Matrix33 bulbR = hrp::rotFromRpy(0,-M_PI/2,0);
        hrp::Matrix33 R = hrp::rotFromRpy(i_cfg[4], i_cfg[5], i_cfg[6]); 
        hrp::Matrix33 wristR(bulbR*R);

        hrp::Vector3 wristP(m_goalP + wristR*hrp::Vector3(-0.03,0,0.18));
        m_reachedArm=0;
        if (!calcIKwithLimitCheck(m_arm[0],wristP, wristR)) {
            m_ikFailCount[0]++;
            m_reachedArm = 1;
            if (!calcIKwithLimitCheck(m_arm[1], wristP, wristR)){
                m_ikFailCount[1]++;
                return false;
            }
        }
        for (int i=0; i<2; i++){ 
            if (!calcIKwithLimitCheck(m_leg[i], m_footP[i], m_footR[i])){
                m_ikFailCount[i+2]++; 
                return false;
            }
        }
        hrp::JointPathPtr path = m_reachedArm == 0 ? m_arm[1] : m_arm[0];
        for (int i=0; i<path->numJoints(); i++){
            hrp::Link *j = path->joint(i);
            j->q = m_q[j->jointId];
        }
        m_body->calcForwardKinematics();
        return true;
    }
    void profile(){
        for (int i=0; i<4; i++) std::cout << m_ikFailCount[i] << " ";
        std::cout << std::endl;
    }
    int reachedArm(){ return m_reachedArm; }
private:
    int m_ikFailCount[4];
    hrp::JointPathPtr m_arm[2], m_trunk;
    hrp::Vector3 m_goalP;
    hrp::dvector m_q;
    int m_reachedArm;
};
