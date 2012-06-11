#include "cfgSetterBase.h"

class myCfgSetter : public cfgSetterBase
{
public:
    myCfgSetter(hrp::BodyPtr i_body)
        : cfgSetterBase(i_body){
        hrp::Link *chest = m_body->link("CHEST_JOINT1");
        hrp::Link *wrist[2] = {m_body->link("RARM_JOINT5"),
                               m_body->link("LARM_JOINT5")};
        for (int i=0; i<2; i++){
            m_arm[i] = m_body->getJointPath(chest, wrist[i]);
            m_ikFailCount[i] = 0;
        }
    }
    bool set(PathEngine::PathPlanner *i_planner,
             const PathEngine::Configuration &i_cfg){
        m_nCalls++;
        setBase(i_cfg[0], i_cfg[1], i_cfg[2], i_cfg[3]);
        for (int k=0; k<2; k++){
            for (int i=0; i<m_arm[k]->numJoints(); i++){
                hrp::Link *j = m_arm[k]->joint(i);
                j->q = i_cfg[4+k*6+i];
            }
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
    hrp::JointPathPtr m_arm[2];
};
