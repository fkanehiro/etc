#include <iostream>
#include <fstream>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/Link.h>
#include <hrpModel/JointPath.h>
#include <hrpModel/ColdetLinkPair.h>

using namespace hrp;
using namespace OpenHRP;

void gen_colmap(BodyPtr body, Link *link1, Link *link2)
{
    JointPathPtr path = body->getJointPath(link1, link2);
    if (path->numJoints() != 2){
        std::cerr << "the number of joints between " << link1->name << " and "
                  << link2->name << "isn't equal to 2" << std::endl;
        return;
    }
    ColdetLinkPairPtr pair = new ColdetLinkPair(link1, link2);

    Link *joint1=path->joint(0), *joint2=path->joint(1);
    double th1 = joint1->llimit, th2;
#define DTH (M_PI/180);
    std::string fname = link1->name + "_" + link2->name + ".dat";
    std::ofstream ofs(fname.c_str());
    ofs << joint1->ulimit << " " << joint2->ulimit << std::endl;
    ofs << joint1->ulimit << " " << joint2->llimit << std::endl;
    ofs << joint1->llimit << " " << joint2->llimit << std::endl;
    ofs << joint1->llimit << " " << joint2->ulimit << std::endl;
    ofs << joint1->ulimit << " " << joint2->ulimit << std::endl;
    ofs << std::endl << std::endl;
    int ntest=0, ncollision=0;
    while (th1 < joint1->ulimit){
        joint1->q = th1;
        th2 = joint2->llimit;
        while (th2 < joint2->ulimit){
            joint2->q = th2;
            path->calcForwardKinematics();
            joint1->coldetModel->setPosition(joint1->attitude(), joint1->p);
            joint2->coldetModel->setPosition(joint2->attitude(), joint2->p);
	    ntest++;
            if (pair->checkCollision()){
                ofs << th1 << " " << th2 << std::endl;
		ncollision++;
	    }
            th2 += DTH;
        }
        th1 += DTH;
    }
    std::cout << link1->name << "<->" << link2->name << ":" << ncollision
	      << "/" << ntest << std::endl;
}

int main(int argc, char *argv[])
{
    if (argc < 2){
        std::cerr << "Usage: " << argv[0] << " vrml [link1] [link2]" << std::endl;
        return 1;
    }

    BodyPtr body(new Body());
    if (!loadBodyFromModelLoader(body, argv[1], argc, argv, true)){
        std::cerr << "failed to load " << argv[1] << std::endl;
        return 1;
    }

    if (argc == 4){
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
      gen_colmap(body, link1, link2);
    }else{
      for (unsigned int i=0; i<body->numLinks(); i++){
	Link *link1  = body->link(i);
        for (unsigned int j=i+1; j<body->numLinks(); j++){
          Link *link2 = body->link(j);
          if (body->getJointPath(link1, link2)->numJoints()  == 2){
            gen_colmap(body, link1, link2);
          }
        }
      }
    }

    return 0;
}
