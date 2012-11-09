#include <hrpModel/Link.h>
#include <Math/Physics.h>
#include "RobotUtil.h"

using namespace motion_generator;
using namespace hrp;

std::ostream& operator<<(std::ostream &ost, HumanoidBodyPtr robot)
{
    for (int j=0; j<3; j++){
        ost << robot->rootLink()->p[j] << " ";
    }
    for (int j=0; j<3; j++){
        for (int k=0; k<3; k++){
            ost << robot->rootLink()->R(j,k) << " ";
        }
    }
    for (int j=0; j<robot->numJoints(); j++){
        ost << robot->joint(j)->q << " ";
    }
    return ost;
}

std::istream& operator>>(std::istream &ist, HumanoidBodyPtr robot)
{
    for (int j=0; j<3; j++){
        ist >> robot->rootLink()->p[j];
    }
    for (int j=0; j<3; j++){
        for (int k=0; k<3; k++){
            ist >> robot->rootLink()->R(j,k);
        }
    }
    for (int j=0; j<robot->numJoints(); j++){
        ist >> robot->joint(j)->q;
    }
    return ist;
}

void setHalfConf(HumanoidBodyPtr robot)
{
    // set halfconf
    dvector halfconf;
    halfconf.setZero(robot->numJoints());
    halfconf[2] = halfconf[ 8] = ToRad(-40);
    halfconf[3] = halfconf[ 9] = ToRad( 78);
    halfconf[4] = halfconf[10] = ToRad(-38);
    double leg_link_len1=0, leg_link_len2=0; 
    halfconf[16] = halfconf[23] = ToRad(20);
    halfconf[17] = ToRad(-10); halfconf[24] = -halfconf[17];
    halfconf[19] = halfconf[26] = ToRad(-30);
    halfconf[20] = ToRad(80); halfconf[27] = -halfconf[20];
    halfconf[22] = halfconf[29] = -1.0;
    leg_link_len1 = leg_link_len2 = 0.3;
    double waistHeight = leg_link_len1*cos(halfconf[2])
        + leg_link_len2*cos(halfconf[4]) 
        + robot->FootToAnkle()[2];
    robot->rootLink()->p << 0,0,waistHeight;
    robot->rootLink()->R.setIdentity();
    for (int i=0; i<robot->numJoints(); i++){
        robot->joint(i)->q = halfconf[i];
        robot->joint(i)->defaultJointValue = halfconf[i];
    }
    robot->calcForwardKinematics();

    Vector3 v(0.03,0,-0.18);
    robot->setGraspLocalPosition(RIGHT, v);
    robot->setGraspLocalPosition(LEFT,  v);
}

