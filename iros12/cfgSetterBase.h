#ifndef __CFG_SETTER_BASE_H__
#define __CFG_SETTER_BASE_H__
#include <Model/HumanoidBodyUtil.h>

class cfgSetterBase
{
public:
    cfgSetterBase(motion_generator::HumanoidBodyPtr i_body) 
        : m_body(i_body), m_nCalls(0){
        for (int i=0; i<2; i++){
            m_leg[i] = m_body->getJointPath(m_body->rootLink(),
                                            m_body->ankleLink[i]);
            m_footP[i] = m_body->ankleLink[i]->p;
            m_footR[i] = m_body->ankleLink[i]->R;
        }
        m_body->calcCM();
        m_body->rootLink()->calcSubMassCM();
        double ubm = m_body->rootLink()->m + m_body->waistYjoint->subm;
        m_cog = (m_body->rootLink()->m*m_body->rootLink()->wc
                 + m_body->waistYjoint->submwc)/ubm;
        hrp::Link *base = m_body->rootLink();
        m_cog2waist = base->R.transpose()*(base->p - m_cog);
        //std::cout << "cog2waist:" << m_cog2waist << std::endl;
    }
    void setBase(double z, double r, double p, double y){
        m_body->rootLink()->R = hrp::rotFromRpy(r,p,y);
        m_body->rootLink()->p = m_cog + m_body->rootLink()->R*m_cog2waist;
        m_body->rootLink()->p[2] = z;
    }
protected:
    motion_generator::HumanoidBodyPtr m_body;
    hrp::Vector3 m_cog, m_cog2waist;
    hrp::JointPathPtr m_leg[2];
    hrp::Vector3 m_footP[2];
    hrp::Matrix33 m_footR[2];
    int m_nCalls;
};

bool calcIKwithLimitCheck(hrp::JointPathPtr i_path,
                          const hrp::Vector3 &i_p, const hrp::Matrix33 &i_R){
    if (!i_path->calcInverseKinematics(i_p, i_R)) {
        return false;
    }
    // check joint limits
    for (int i=0; i<i_path->numJoints(); i++){
        hrp::Link *j = i_path->joint(i);
        if (j->q > j->ulimit || j->q < j->llimit){
            return false;
        }
    }
    return true;
}
#endif
