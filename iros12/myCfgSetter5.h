/*
  ランダムサンプリングした胴体の高さ、回転、手先の回転から全関節角度を決定する
  使用する腕、ヨー角は固定
  コンフィギュレーションの長さは4+2
 */
#include <Math/MathFunction.h>
#include "cfgSetterBase.h"

class myCfgSetter5 : public cfgSetterBase
{
public:
    myCfgSetter5(motion_generator::HumanoidBodyPtr i_body, int i_arm,
                 const hrp::Vector3 &i_goalP, double i_yaw)
        : cfgSetterBase(i_body), m_reachingArm(i_arm), m_goalP(i_goalP), m_yaw(i_yaw), m_q(i_body->numJoints()){
        for (int i=0; i<2; i++){
            m_arm[i] = m_body->getJointPath(m_body->chestLink,
                                            m_body->wristLink[i]);
        }
        m_trunk = m_body->getJointPath(m_body->rootLink(), m_body->chestLink);
        for (int i=0; i<3; i++){
            m_ikFailCount[i] = 0;
        }
        m_body->getPosture(m_q);
        //std::cout << "arm = " << m_reachingArm << ", yaw = " << m_yaw << std::endl;
    }
    bool set(PathEngine::PathPlanner *i_planner,
             const PathEngine::Configuration &i_cfg){
        m_nCalls++; 
        // compute trunk horizontal position
        setBase(i_cfg[0], i_cfg[1], i_cfg[2], i_cfg[3]);
        m_trunk->calcForwardKinematics();

        hrp::Matrix33 bulbR = hrp::rotFromPitch(-M_PI/2);
        hrp::Matrix33 R = hrp::rotFromRpy(i_cfg[4], i_cfg[5], m_yaw); 
        hrp::Matrix33 wristR(bulbR*R);

        hrp::Vector3 wristP(m_goalP - wristR*m_body->getGraspLocalPosition(0));
        if (!calcIKwithLimitCheck(m_arm[m_reachingArm],wristP, wristR)) {
            m_ikFailCount[0]++;
            return false;
        }
        for (int i=0; i<2; i++){ 
            if (!calcIKwithLimitCheck(m_leg[i], m_footP[i], m_footR[i])){
                m_ikFailCount[i+1]++; 
                return false;
            }
        }
        int freeArm = m_reachingArm == 0 ? 1 : 0;
        for (int i=0; i<m_arm[freeArm]->numJoints(); i++){
            hrp::Link *j = m_arm[freeArm]->joint(i);
            j->q = m_q[j->jointId];
        }
        m_body->calcForwardKinematics();
        return true;
    }
    void profile(){
        std::cout << m_nCalls << ",";
        for (int i=0; i<3; i++) std::cout << m_ikFailCount[i] << " ";
        std::cout << std::endl;
    }
private:
    int m_ikFailCount[3];
    int m_reachingArm;
    hrp::JointPathPtr m_arm[2], m_trunk;
    hrp::Vector3 m_goalP;
    double m_yaw;
    hrp::dvector m_q;
};
