#include <iostream>
#include <fstream>
#include <Math/Physics.h>
#include <hrpUtil/OnlineViewerUtil.h>
#include <Util/OnlineViewerUtil.h>
#include <Model/HumanoidBodyUtil.h>
#include <hrpModel/JointPath.h>
#include <sys/time.h>
#define IKFAST_HAS_LIBRARY
#include <ikfast.h>
using namespace ikfast;

using namespace motion_generator;

void updateView(OpenHRP::OnlineViewer_var olv,
		OpenHRP::WorldState &wstate, 
		HumanoidBodyPtr& body)
{
  body->calcForwardKinematics();
  updateCharacterPosition(wstate.characterPositions[0], body);
  olv->update(wstate);
  wstate.time += 0.01;
}

int main(int argc, char *argv[])
{
  if (argc < 2){
    std::cerr << "Usage:" << argv[0] << "[-checkLimits]" 
	      << std::endl;
  }

  const char *url="file:///home/kanehiro/src/HRP2/model/HRP2main.wrl";
  int arm = 0;
  bool checkLimits=false;
  bool display=true;
  for (int i=1; i<argc; i++){
    if(strcmp(argv[i], "-checkLimits")==0){
      checkLimits = true;
    }else if(strcmp(argv[i], "-nodisplay")==0){
      display = false;
    }
  }
  std::cout << "arm=" << arm << std::endl;
  std::cout << "checkLimits=" << checkLimits << std::endl;


  HumanoidBodyPtr body = HumanoidBodyPtr(new HumanoidBody());
  if (!loadHumanoidBodyFromModelLoader(body, url, argc, argv)){
    std::cerr << "failed to load model[" << url << "]" 
	      << std::endl;
    return 1;
  }

  OpenHRP::OnlineViewer_var olv;
  if (display){
    olv = hrp::getOnlineViewer(argc, argv);
    olv->load(body->name().c_str(), url);

    olv->clearLog();
  }
  
  OpenHRP::WorldState wstate;
  wstate.time = 0.0;
  wstate.characterPositions.length(1);
  setupCharacterPosition(wstate.characterPositions[0], body);
  wstate.collisions.length(1);

  hrp::JointPathPtr jp;
  if (arm == 0){
    jp = body->getJointPath(body->link("CHEST_JOINT1"),
			    body->link("RARM_JOINT5"));
  }else{
    jp = body->getJointPath(body->link("CHEST_JOINT1"),
			    body->link("LARM_JOINT5"));
  }
  hrp::Vector3 p;
  hrp::Matrix33 R = hrp::rotFromRpy(0, -M_PI/2, 0);

  double startx=-0.3, starty=-1.0, startz=1.0;
  double endx=0.7, endy=0.5, endz=2.0;
  if (arm != 0){
    double tmp = starty;
    starty = -endy;
    endy = -tmp;
  }

  wstate.collisions[0].points.length(12);
  updateLineSegment(wstate.collisions[0].points[0], hrp::Vector3(startx, starty, startz), hrp::Vector3(endx, starty, startz));
  updateLineSegment(wstate.collisions[0].points[1], hrp::Vector3(startx, starty, startz), hrp::Vector3(startx, endy, startz));
  updateLineSegment(wstate.collisions[0].points[2], hrp::Vector3(startx, starty, startz), hrp::Vector3(startx, starty, endz));
  updateLineSegment(wstate.collisions[0].points[3], hrp::Vector3(endx, endy, startz), hrp::Vector3(startx, endy, startz));
  updateLineSegment(wstate.collisions[0].points[4], hrp::Vector3(endx, endy, startz), hrp::Vector3(endx, starty, startz));
  updateLineSegment(wstate.collisions[0].points[5], hrp::Vector3(endx, endy, startz), hrp::Vector3(endx, endy, endz));
  updateLineSegment(wstate.collisions[0].points[6], hrp::Vector3(endx, starty, endz), hrp::Vector3(endx, starty, startz));
  updateLineSegment(wstate.collisions[0].points[7], hrp::Vector3(endx, starty, endz), hrp::Vector3(startx, starty, endz));
  updateLineSegment(wstate.collisions[0].points[8], hrp::Vector3(endx, starty, endz), hrp::Vector3(endx, endy, endz));
  updateLineSegment(wstate.collisions[0].points[9], hrp::Vector3(startx, endy, endz), hrp::Vector3(endx, endy, endz));
  updateLineSegment(wstate.collisions[0].points[10], hrp::Vector3(startx, endy, endz), hrp::Vector3(startx, endy, startz));
  updateLineSegment(wstate.collisions[0].points[11], hrp::Vector3(startx, endy, endz), hrp::Vector3(startx, starty, endz));
  double step=0.05;
  int total=0, success=0;

  IkSolutionList<IkReal> solutions;
  IkReal eerot[9],eetrans[3];
  std::vector<IkReal> solvalues(GetNumJoints());
  double linErr=0, angErr=0;

  struct timeval begin, end;
  gettimeofday(&begin, NULL);

  for (double x=startx; x<endx; x+=step){
    for (double y=starty; y<endy; y+=step){
      for (double z=startz; z<endz; z+=step){
	total++;
	p << x,y,z;
	if (display) std::cout << "testing " << p.transpose() << " - ";
#if 0
	bool ret = jp->calcInverseKinematics(p, R);

	if (ret && checkLimits){
	  for (int i=0; i<jp->numJoints(); i++){
	    hrp::Link *j = jp->joint(i);
	    if (j->q > j->ulimit || j->q < j->llimit){
	      std::cout << "fail\r" << std::flush;
	      ret = false;
	      break;
	    }
	  }
	}
#else
	hrp::Vector3 relP;
	hrp::Matrix33 relR;
	relR = jp->baseLink()->R.transpose() * R;
	relP = jp->baseLink()->R.transpose() * (p - jp->baseLink()->p);
	for (int i=0; i<3; i++){
	  eetrans[i] = relP[i];
	  for (int j=0; j<3; j++){
	    eerot[i*3+j] = relR(i,j);
	  }
	}
	bool ret = ComputeIk(eetrans, eerot, NULL, solutions);
	if (ret) {
	  ret = false;
	  for (int j=0; j<solutions.GetNumSolutions(); j++){
	    const IkSolutionBase<IkReal>& sol = solutions.GetSolution(j);
  	    sol.GetSolution(&solvalues[0],NULL);
	    bool found=true;
	    for (int i=0; i<jp->numJoints(); i++){
	      hrp::Link *l = jp->joint(i);
	      if (solvalues[i] > l->ulimit || solvalues[i] < l->llimit) {
		found = false;
		break;
	      }else{
		l->q = solvalues[i];
	      }
	    }
	    if (found) {
	      ret = true;
	      jp->calcForwardKinematics();
	      double perr = (jp->endLink()->p - p).norm();
	      if (perr > linErr) linErr = perr;
	      double ang = hrp::omegaFromRot(jp->endLink()->R.transpose()*R).norm();
	      if (ang > angErr) angErr = ang;
	      break;
	    }
	  }
	}
	if (!ret && display){
	  std::cout << "fail\r" << std::flush;
	}
#endif
	if (ret) success++;

	if (display){
	  if (ret){
	    std::cout << "success\r" << std::flush;
	    unsigned int len = wstate.collisions[0].points.length();
	    wstate.collisions[0].points.length(len+1);
	    updateLineSegment(wstate.collisions[0].points[len],
			      p, R*hrp::Vector3(0,0,-0.01)+p);
	    updateView(olv, wstate, body);
	  }else{
	    std::cout << "fail\r" << std::flush;
	  }
	}
      }
    }
  }

  gettimeofday(&end, NULL);

  std::cout << std::endl;
  std::cout << "result:" << success << "/" << total << "(" 
	    << ((double)success)/total*100 << "%)" << std::endl;

  std::cout << "computation time:" << (end.tv_sec - begin.tv_sec)*1e3 + (end.tv_usec - begin.tv_usec)*1e-3 << "[ms]" << std::endl;
  std::cout << "maximum err:" << linErr << "[m], " << angErr << "[rad]" << std::endl; 

  return 0;
}
