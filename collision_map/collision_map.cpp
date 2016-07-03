#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/Link.h>
#include <hrpCollision/ColdetModelPair.h>
#include <iostream>

int main(int argc, char *argv[])
{
  if (argc < 4){
    std::cerr << "Usage: " << argv[0] 
	      << "[VRML model] [joint name1] [joint name2]" << std::endl;
    return 1;
  }

  hrp::BodyPtr body(new hrp::Body());

  if (!loadBodyFromModelLoader(body, argv[1], argc, argv, true)){
    std::cerr << "failed to load model" << std::endl;
    return 1;
  }

  hrp::Link *joint1 = body->link(argv[2]);
  if (!joint1){
    std::cerr << "can't find a joint named " << argv[2] << std::endl;
    return 1;
  }
  hrp::Link *joint2 = body->link(argv[3]);
  if (!joint2){
    std::cerr << "can't find a joint named " << argv[3] << std::endl;
    return 1;
  }

  hrp::ColdetModelPair pair(joint1->parent->coldetModel, joint2->coldetModel); 

  double dth=0.01;
  double th1 = joint1->llimit;
  while (th1 < joint1->ulimit){
    joint1->q = th1;
    double th2 = joint2->llimit;
    while (th2 < joint2->ulimit){
      joint2->q = th2;
      body->calcForwardKinematics();
      joint1->parent->coldetModel->setPosition(joint1->parent->R, joint1->parent->p);
      joint2->coldetModel->setPosition(joint2->R, joint2->p);
      if (pair.checkCollision()){
	std::cout << th1 << " " << th2 << std::endl;
      }
      th2 += dth;
    }
    th1 += dth;
  }




  return 0;
}
