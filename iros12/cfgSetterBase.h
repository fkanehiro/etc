#ifndef __CFG_SETTER_BASE_H__
#define __CFG_SETTER_BASE_H__
#include <Model/HumanoidBodyUtil.h>
#include <hrpModel/Link.h>
#include <hrpModel/JointPath.h>
#include <iostream>
#include <hrpPlanner/PathPlanner.h>
#define USE_ANALYTICAL_IK

#ifndef USE_ANALYTICAL_IK
#include <hrpUtil/MatrixSolvers.h>

bool calcInverseKinematics(hrp::JointPathPtr jpath, const hrp::Vector3& end_p, const hrp::Matrix33& end_R)
{
    static const int MAX_IK_ITERATION = 50;
    static const double LAMBDA = 0.9;
    static const double maxIKErrorSqr = 1.0e-6 * 1.0e-6;
    double prevError;

    const int n = jpath->numJoints();

    hrp::Link* target = jpath->endLink();

    hrp::dmatrix J(6, n);
    hrp::dvector dq(n);
    hrp::dvector v(6);

    double errsqr = maxIKErrorSqr * 100.0;
    for (int i=0; i<n; i++){
        hrp::Link *j = jpath->joint(i);
        if (j) j->q = j->defaultJointValue;
    }
    jpath->calcForwardKinematics();

    for(int i=0; i < MAX_IK_ITERATION; i++){
	
        jpath->calcJacobian(J);
	
        hrp::Vector3 dp(end_p - target->p);
        hrp::Vector3 omega(target->R * hrp::omegaFromRot(target->R.transpose() * end_R));

        const double errsqr = dp.dot(dp) + omega.dot(omega);
        //std::cout << i << ":" << errsqr << std::endl;
        if (i==0) prevError = errsqr;
        if (errsqr > prevError) return false;
        if(errsqr < maxIKErrorSqr){
            return true;
        }

        v << dp, omega;
		
        if(n == 6){ 
            hrp::solveLinearEquationLU(J, v, dq);
        } else {
            hrp::solveLinearEquationSVD(J, v, dq);  // dq = pseudoInverse(J) * v
        }
		
        for(int j=0; j < n; ++j){
            jpath->joint(j)->q += LAMBDA * dq(j);
        }

        jpath->calcForwardKinematics();
        prevError = errsqr;
    }

    return false;
}
#endif

class cfgSetterBase
{
public:
    cfgSetterBase(motion_generator::HumanoidBodyPtr i_body) 
        : m_body(i_body), m_nCalls(0){
#ifndef USE_ANALYTICAL_IK
        for (int i=0; i<m_body->numJoints(); i++){
            hrp::Link *j = m_body->joint(i);
            if (j) j->defaultJointValue = j->q;
        }
#endif
        for (int i=0; i<2; i++){
            m_leg[i] = m_body->getJointPath(m_body->rootLink(),
                                            m_body->ankleLink[i]);
            m_footP[i] = m_body->ankleLink[i]->p;
            m_footR[i] = m_body->ankleLink[i]->R;
        }
        resetCog();
    }
    void setBase(double z, double r, double p, double y){
        m_body->rootLink()->R = hrp::rotFromRpy(r,p,y);
        m_body->rootLink()->p = m_cog + m_body->rootLink()->R*m_cog2waist;
        m_body->rootLink()->p[2] = z;
    }
    void resetCog(){
        m_body->calcCM();
        m_body->rootLink()->calcSubMassCM();
        double ubm = m_body->rootLink()->m + m_body->waistYjoint->subm;
        m_cog = (m_body->rootLink()->m*m_body->rootLink()->wc
                 + m_body->waistYjoint->submwc)/ubm;
        hrp::Link *base = m_body->rootLink();
        m_cog2waist = base->R.transpose()*(base->p - m_cog);
        //std::cout << "cog:" << m_cog.transpose() << std::endl;
        //std::cout << "cog2waist:" << m_cog2waist.transpose() << std::endl;
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
#ifdef USE_ANALYTICAL_IK
    if (!i_path->calcInverseKinematics(i_p, i_R)) {
        return false;
    }
#else
    if (!calcInverseKinematics(i_path, i_p, i_R)) {
        return false;
    }
#endif
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
