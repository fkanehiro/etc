#include "cfgSetterBase.h"

class myCfgSetter : public cfgSetterBase
{
public:
    myCfgSetter(motion_generator::HumanoidBodyPtr i_body, int i_arm)
        : cfgSetterBase(i_body){
        m_arm = m_body->getJointPath(m_body->chestLink,
                                     m_body->wristLink[i_arm]);
        for (int i=0; i<2; i++){
            m_ikFailCount[i] = 0;
        }
    }
    bool set(PathEngine::PathPlanner *i_planner,
             const PathEngine::Configuration &i_cfg){
        m_nCalls++;
        setBase(i_cfg[0], i_cfg[1], i_cfg[2], i_cfg[3]);
        for (int i=0; i<m_arm->numJoints(); i++){
            hrp::Link *j = m_arm->joint(i);
            j->q = i_cfg[4+i];
        }
        for (int i=0; i<2; i++){ 
            if (!calcIKwithLimitCheck(m_leg[i],m_footP[i], m_footR[i])){
                m_ikFailCount[i]++; 
                return false;
            }
        }
        m_body->calcForwardKinematics();
        return true;
    }
    void profile(){
        std::cout << m_nCalls << " times called( ";
        for (int i=0; i<2; i++) std::cout << m_ikFailCount[i] << " ";
        std::cout << ")" << std::endl;
    }
private:
    int m_ikFailCount[2];
    hrp::JointPathPtr m_arm;
};
