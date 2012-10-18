/*
  ランダムサンプリングした胴体の高さ、回転、手先の高さ、ピッチ軸回りの回転から
  全関節角度を決定する
 */
#include <Math/MathFunction.h>
#include "cfgSetterBase.h"

class myCfgSetter : public cfgSetterBase
{
public:
    myCfgSetter(motion_generator::HumanoidBodyPtr i_body,
                const hrp::Vector3 &i_goalP)
        : cfgSetterBase(i_body), m_goalP(i_goalP), m_q(i_body->numJoints()){
        for (int i=0; i<2; i++){
            m_arm[i] = m_body->getJointPath(m_body->chestLink,
                                            m_body->wristLink[i]);
        }
        m_trunk = m_body->getJointPath(m_body->rootLink(), m_body->chestLink);
        for (int i=0; i<4; i++){
            m_ikFailCount[i] = 0;
        }
        m_body->getPosture(m_q);
    }
    bool set(PathEngine::PathPlanner *i_planner,
             const PathEngine::Configuration &i_cfg){
        // compute trunk horizontal position
        setBase(i_cfg[0], i_cfg[1], i_cfg[2], i_cfg[3]);
        m_trunk->calcForwardKinematics();

        hrp::Matrix33 R = hrp::rotFromRpy(0, i_cfg[5], 0); 
        hrp::Vector3 goalP(m_goalP);
        goalP[2] += i_cfg[4];
        //goalP[1] += i_cfg[4];
        hrp::Matrix33 Rp(hrp::rotFromPitch(-M_PI/2));

        m_reachedArm=0;
        hrp::Matrix33 bulbR(Rp*hrp::rotFromYaw(M_PI/2));
        //hrp::Matrix33 bulbR(Rp);
        hrp::Matrix33 wristR(bulbR*R);
        hrp::Vector3 wristP(goalP + wristR*hrp::Vector3(-0.03,0,0.18));
        if (!calcIKwithLimitCheck(m_arm[0],wristP, wristR)) {
            m_ikFailCount[0]++;
            m_reachedArm = 1;
            hrp::Matrix33 bulbR(Rp*hrp::rotFromYaw(-M_PI/2));
            //hrp::Matrix33 bulbR(Rp);
            hrp::Matrix33 wristR(bulbR*R);
            hrp::Vector3 wristP(goalP + wristR*hrp::Vector3(-0.03,0,0.18));
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
