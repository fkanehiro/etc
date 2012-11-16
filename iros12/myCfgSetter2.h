/*
  ランダムサンプリングした、体幹の高さ、回転、腕の関節角度から全関節角度を
　決定する
 */
#include "cfgSetterBase.h"

class myCfgSetter2 : public cfgSetterBase
{
public:
    myCfgSetter2(motion_generator::HumanoidBodyPtr i_body)
        : cfgSetterBase(i_body){
        for (int i=0; i<2; i++){
            m_arm[i] = m_body->getJointPath(m_body->chestLink,
                                            m_body->wristLink[i]);
            m_ikFailCount[i] = 0;
        }
    }
    bool set(PathEngine::PathPlanner *i_planner,
             const PathEngine::Configuration &i_cfg){
        m_nCalls++;
        setBase(i_cfg[0], i_cfg[1], i_cfg[2], i_cfg[3]);
        for (int i=0; i<2; i++){ 
            if (!calcIKwithLimitCheck(m_leg[i],m_footP[i], m_footR[i])){
                m_ikFailCount[i]++; 
                return false;
            }
        }
        int armDof = m_arm[0]->numJoints();
        for (int k=0; k<2; k++){
            for (int i=0; i<armDof; i++){
                hrp::Link *j = m_arm[k]->joint(i);
                j->q = i_cfg[4+k*armDof+i];
            }
        }
        m_body->calcForwardKinematics();
        return true;
    }
    void profile(){
        std::cout << m_nCalls << " calls,";
        for (int i=0; i<2; i++) std::cout << m_ikFailCount[i] << " ";
        std::cout << std::endl;
    }
private:
    int m_ikFailCount[2];
    hrp::JointPathPtr m_arm[2];
};
