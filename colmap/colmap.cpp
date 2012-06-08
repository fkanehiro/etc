#include <iostream>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/Link.h>
#include <hrpModel/JointPath.h>
#include <hrpModel/ColdetLinkPair.h>

using namespace hrp;
using namespace OpenHRP;

int main(int argc, char *argv[])
{
    if (argc < 4){
        std::cerr << "Usage: " << argv[0] << " vrml link1 link2" << std::endl;
        return 1;
    }

    BodyPtr body = new Body();
    if (!loadBodyFromModelLoader(body, argv[1], argc, argv, true)){
        std::cerr << "failed to load " << argv[1] << std::endl;
        return 1;
    }
    Link *link1, *link2;
    link1 = body->link(argv[2]);
    if (!link1){
        std::cerr << "can't find a link named " << argv[2] << std::endl;
        return 1;
    }
    link2 = body->link(argv[3]);
    if (!link2){
        std::cerr << "can't find a link named " << argv[3] << std::endl;
        return 1;
    }

    JointPathPtr path = body->getJointPath(link1, link2);
    if (path->numJoints() != 2){
        std::cerr << "the number of joints between " << argv[2] << " and "
                  << argv[3] << "doesn't equal to 2" << std::endl;
        return 1;
    }
    ColdetLinkPairPtr pair = new ColdetLinkPair(link1, link2);

    Link *joint1=path->joint(0), *joint2=path->joint(1);
    double th1 = joint1->llimit, th2;
#define DTH (M_PI/180);
    while (th1 < joint1->ulimit){
        joint1->q = th1;
        th2 = joint2->llimit;
        while (th2 < joint2->ulimit){
            joint2->q = th2;
            path->calcForwardKinematics();
            link1->coldetModel->setPosition(link1->attitude(), link1->p);
            link2->coldetModel->setPosition(link2->attitude(), link2->p);
            if (!pair->checkCollision()){
                std::cout << th1 << " " << th2 << std::endl;
            }
            th2 += DTH;
        }
        th1 += DTH;
    }
    return 0;
}
