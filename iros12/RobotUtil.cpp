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
    if (robot->modelName() == "HRP4Cg"){
        halfconf[2] = halfconf[ 9] = ToRad(-40);
        halfconf[3] = halfconf[10] = ToRad( 78);
        halfconf[4] = halfconf[11] = ToRad(-38);
    }else{
        halfconf[2] = halfconf[ 8] = ToRad(-40);
        halfconf[3] = halfconf[ 9] = ToRad( 78);
        halfconf[4] = halfconf[10] = ToRad(-38);
    }
    double leg_link_len1=0, leg_link_len2=0; 
    if (robot->modelName() == "HRP2SH"){
        halfconf[16] = halfconf[24] = ToRad(20);
        halfconf[17] = ToRad(-10); halfconf[25] = -halfconf[17];
        halfconf[19] = halfconf[27] = ToRad(-30);
        halfconf[23] = halfconf[31] = 0.5;
        halfconf[32] = halfconf[37] = -0.5;
        halfconf[33] = halfconf[38] = 0.5;
        halfconf[34] = halfconf[39] = -0.5;
        halfconf[35] = halfconf[40] = 0.5;
        halfconf[36] = halfconf[41] = -0.5;
    }else if (robot->modelName() == "HRP4Cg"){
        halfconf[28] = halfconf[36] = ToRad(20);
        halfconf[29] = ToRad(-10); halfconf[37] = -halfconf[17];
        halfconf[31] = halfconf[39] = ToRad(-30);
    }else{
        halfconf[16] = halfconf[23] = ToRad(20);
        halfconf[17] = ToRad(-10); halfconf[24] = -halfconf[17];
        halfconf[19] = halfconf[26] = ToRad(-30);
        halfconf[20] = ToRad(80); halfconf[27] = -halfconf[20];
        halfconf[22] = halfconf[29] = -1.0;
    }
    if (robot->modelName() == "HRP4Cg"){
        leg_link_len1 = 0.358;
        leg_link_len2 = 0.3411509929635264;
    }else{
        leg_link_len1 = leg_link_len2 = 0.3;
    }
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

    Vector3 v;
    if (robot->modelName() == "HRP2SH"){
        v << 0.0,0,-0.18;
    }else{
        v << 0.03,0,-0.18;
    }
    robot->setGraspLocalPosition(RIGHT, v);
    robot->setGraspLocalPosition(LEFT,  v);
}

