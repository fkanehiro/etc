/*
  バルブ回転動作の探索
 */
#include <fstream>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <Model/HumanoidBodyUtil.h>
#include <hrpModel/JointPath.h>
#include <hrpModel/Link.h>
#include <hrpPlanner/RRT.h>
#include <hrpPlanner/Roadmap.h>
#include <hrpPlanner/RoadmapNode.h>
#include <hrpPlanner/RandomShortcutOptimizer.h>
#include <hrpPlanner/ShortcutOptimizer.h>
#include <hrpPlanner/ConfigurationSpace.h>
#include "problem.h"
#include "myCfgSetter2.h" // setter for intermediate goal
#include "myCfgSetter3.h" // setter for path from intermediate goal to final one
#include "myCfgSetter5.h" // setter for final goal
#include <Math/Physics.h>
#include "CustomCD.h"
#include "RobotUtil.h"

using namespace motion_generator;
using namespace hrp;
using namespace PathEngine;

template<class T>
bool testRandomConfig(PathPlanner *planner, T &setter, 
                      ConfigurationSpace& cspace, 
                      Configuration& cfg)
{
    cfg = cspace.random();
    return setter.set(planner, cfg) && !planner->checkCollision();
}

class subproblem
{
public:
    subproblem(problem *mainproblem_, CollisionDetector *cd,
               HumanoidBodyPtr robot,
               const Configuration& startCfg, int arm, Vector3& goalP,
               double angle) :
        mainproblem(mainproblem_), isSolved(false),
        CSforGoal(4+2),
        setterForGoal(robot, arm, goalP, startCfg[6]+angle),
        setterForPath(robot, goalP),
        goalCfg(7), usedArm(arm){
        if (goalCfg.size() != startCfg.size()){
            std::cerr << "subproblem: dimension mismatch(" << goalCfg.size()
                      << "<->" << startCfg.size() << ")" << std::endl;
        }
        goalCfg[6] = startCfg[6]+angle;
        planner = new PathPlanner(goalCfg.size(), 
                                  mainproblem->planner()->world());
        planner->setMobilityName("OmniWheel");
        planner->setAlgorithmName("RRT");
        planner->getMobility()->interpolationDistance(0.05);
        planner->setCollisionDetector(cd);
        planner->setApplyConfigFunc(boost::bind(&myCfgSetter3::set, 
                                                &setterForPath, _1, _2));
        RRT *rrt = (RRT *)planner->getAlgorithm();
        rrt->getForwardTree()->addNode(RoadmapNodePtr(new RoadmapNode(startCfg)));
        rrt->extendFromGoal(true);
        rrt->epsilon(0.1);

        // configuration space for goal search
#if 1
#if 1
        CSforGoal.bounds(0,  0.26, 0.705); // body z
        CSforGoal.bounds(1, -0.5, 0.5); // body roll
        CSforGoal.bounds(2, -0.0, 0.5); // body pitch
        CSforGoal.bounds(3, -0.5, 0.5); // body yaw
#else
        for (int i=0; i<4; i++) CSforGoal.bounds(i, startCfg[i]-0.1, startCfg[i]+0.1);
#endif
        //CSforGoal.bounds(4, -M_PI/2, M_PI/2); // hand roll
        CSforGoal.bounds(4, startCfg[4], startCfg[4]);
        //CSforGoal.bounds(5, -M_PI/2, M_PI/2); // hand pitch
        CSforGoal.bounds(5, startCfg[5], startCfg[5]);
#else
        for (int i=0; i<CSforGoal.size(); i++) CSforGoal.bounds(i, startCfg[i], startCfg[i]);
#endif

        // configuration space for path search
        ConfigurationSpace *CSforPath = planner->getConfigurationSpace();
#if 0
        CSforPath->bounds(0,  0.26, 0.705); // body z
        CSforPath->bounds(1, -0.5, 0.5); // body roll
        CSforPath->bounds(2, -0.0, 0.5); // body pitch
        CSforPath->bounds(3, -0.5, 0.5); // body yaw
#else
        for (int i=0; i<4; i++) CSforPath->bounds(i, startCfg[i]-0.1, startCfg[i]+0.1);
#endif
        //CSforPath->bounds(4, -M_PI/2, M_PI/2); // hand roll
        CSforPath->bounds(4, startCfg[4], startCfg[4]);
        //CSforPath->bounds(5, -M_PI/2, M_PI/2); // hand pitch
        CSforPath->bounds(5, startCfg[5], startCfg[5]);
        //CSforPath->bounds(6, -M_PI, M_PI);   // hand yaw
        CSforPath->bounds(6, startCfg[6], goalCfg[6]);   // hand yaw
    }
    bool oneStep(){
        RRT *rrt = (RRT *)planner->getAlgorithm();
        RoadmapPtr Tg = rrt->getBackwardTree();
        double Psample = 0.1;
        if (!Tg->nNodes() || rand() < Psample*RAND_MAX){
            Configuration cfg(CSforGoal.size());
            if (testRandomConfig(mainproblem->planner(), setterForGoal,
                                 CSforGoal, cfg)){
                for (int i=0; i<4+2; i++) goalCfg[i] = cfg[i];
                Tg->addNode(RoadmapNodePtr(new RoadmapNode(goalCfg)));
                //std::cout << "found a final goal:" << goalCfg << std::endl;
                //mainproblem->updateOLV();
            }
        }else{
            if (rrt->extendOneStep()){
                //std::cout << "found a manipulation path" << std::endl;
                isSolved = true;
                return true;
            }
        }
        return false;
    }

    problem *mainproblem;
    PathPlanner *planner;
    bool isSolved;
    ConfigurationSpace CSforGoal;
    myCfgSetter5 setterForGoal;
    myCfgSetter3 setterForPath;
    Configuration goalCfg;
    int usedArm;
};

class reachingRRT
{
public:
    reachingRRT(RRT *i_rrt) : rrt(i_rrt), isSolved(false) {}
    RRT *rrt;
    bool isSolved;
    std::vector<Configuration> path;
};

int main(int argc, char *argv[])
{
    srand((unsigned)time(NULL));
 
    const char *robotURL = NULL;
    const char *goalURL = NULL;
    std::vector<std::string> obstacleURL;
    std::vector<Vector3> obstacleP;
    std::vector<Vector3> obstacleRpy;
    double angle=M_PI/4;
    Vector3 p, rpy;
    bool display = true;
    for(int i = 1 ; i < argc; i++){
        if (strcmp(argv[i], "-robot") == 0){
            robotURL = argv[++i];
        }else if (strcmp(argv[i], "-obstacle") == 0){
            obstacleURL.push_back(argv[++i]);
            for (int j=0; j<3; j++) p[j] = atof(argv[++i]);
            obstacleP.push_back(p);
            for (int j=0; j<3; j++) rpy[j] = atof(argv[++i]);
            obstacleRpy.push_back(rpy);
        }else if (strcmp(argv[i], "-goal") == 0){
            goalURL = argv[++i];
        }else if (strcmp(argv[i], "-angle") == 0){
            angle = atof(argv[++i]);
        }else if (strcmp(argv[i], "-no-display")==0){
            display = false;
        }
    }
    // goal position
    Vector3 goalP, goalRpy;
    for(int i = 1 ; i < argc; i++){
        if (strcmp(argv[i], "-goalpos") == 0){
            for (int j=0; j<3; j++) {
                goalP(j) = atof(argv[++i]);
            }
            for (int j=0; j<3; j++) {
                goalRpy(j) = atof(argv[++i]);
            }
        }
    }
    if (robotURL == NULL){
        std::cerr << "please specify URL of VRML model by -robot option"
                  << std::endl;
        return 1;
    }


    HumanoidBodyPtr robot = HumanoidBodyPtr(new HumanoidBody());
    loadHumanoidBodyFromModelLoader(robot, robotURL, argc, argv, true);

    problem prob(4+6+6);
    prob.addRobot("robot", robotURL, robot);
    std::vector<BodyPtr> obstacles;
    for (unsigned int i=0; i<obstacleURL.size(); i++){
        char buf[20];
        sprintf(buf, "obstacle%02d", i);
        obstacles.push_back(prob.addObstacle(buf, obstacleURL[i]));
    }
    BodyPtr goal;
    if (goalURL){
        goal = prob.addGoal("goal", goalURL);
    }

    // This must be called after all bodies are added
    if (display) prob.initOLV(argc, argv);

    PathPlanner *planner = prob.planner();
    planner->setMobilityName("OmniWheel");
    planner->setAlgorithmName("RRT");
    RRT *rrt = (RRT *)planner->getAlgorithm();
    rrt->extendFromGoal(true);
    rrt->epsilon(0.1);
    planner->getMobility()->interpolationDistance(0.05);
    prob.initCollisionCheckPairs();
    prob.initPlanner();

    for (unsigned int i=0; i<obstacles.size(); i++){
        Link *root = obstacles[i]->rootLink();
        root->p = obstacleP[i];
        root->R = rotFromRpy(obstacleRpy[i]);
        root->coldetModel->setPosition(root->R, root->p);
        obstacles[i]->calcForwardKinematics();
    }

    // set halfconf
    setHalfConf(robot);

    Matrix33 goalR(rotFromRpy(goalRpy));
    if (goal){
        goal->rootLink()->p = goalP;
        goal->rootLink()->R = goalR;
        goal->calcForwardKinematics();
    }

    myCfgSetter3 setterForIntermediateGoal(robot, goalP);
    //myCfgSetter3 setterForPath(robot, goalP);
    myCfgSetter2 setterForPath(robot);

    CustomCD cd(robot, "hrp2.shape", "hrp2.pairs", 
                obstacles[0], "plant.pc");
    planner->setCollisionDetector(&cd);
    
    JointPathPtr armPath[2];
    for (int k=0; k<2; k++){
        armPath[k] = robot->getJointPath(robot->chestLink,
                                         robot->wristLink[k]);
    }

    ConfigurationSpace* CSforPath = planner->getConfigurationSpace();

    CSforPath->bounds(0,  0.26, 0.705); // body z
    CSforPath->bounds(1, -0.5, 0.5); // body roll
    CSforPath->bounds(2, -0.0, 0.5); // body pitch
    CSforPath->bounds(3, -0.5, 0.5); // body yaw
    for (int k=0; k<2; k++){
        for (int i=0; i<armPath[k]->numJoints(); i++){
            Link *j = armPath[k]->joint(i);
            CSforPath->bounds(4+k*6+i, j->llimit, j->ulimit);
        }
    }

    CSforPath->weight(0) = 0.1; // z
    CSforPath->weight(1) = 1;  // roll
    CSforPath->weight(2) = 1;  // pitch
    CSforPath->weight(3) = 1;  // yaw

    CSforPath->weight(4) = 0.8;
    CSforPath->weight(5) = 0.6;
    CSforPath->weight(6) = 0.4;
    CSforPath->weight(7) = 0.3;
    CSforPath->weight(8) = 0.2;
    CSforPath->weight(9) = 0.1;

    CSforPath->weight(10) = 0.8;
    CSforPath->weight(11) = 0.6;
    CSforPath->weight(12) = 0.4;
    CSforPath->weight(13) = 0.3;
    CSforPath->weight(14) = 0.2;
    CSforPath->weight(15) = 0.1;

    Configuration startCfg(CSforPath->size());
    startCfg[0] = robot->rootLink()->p[2];
    startCfg[1] = startCfg[2] = startCfg[3] = 0;
    for (int j=0; j<2; j++){
        for (int i=0; i<armPath[j]->numJoints(); i++){
            startCfg[4+j*6+i] = armPath[j]->joint(i)->q;
        }
    }
    rrt->getForwardTree()->addNode(RoadmapNodePtr(new RoadmapNode(startCfg)));
    planner->setApplyConfigFunc(boost::bind(&myCfgSetter2::set, 
                                            &setterForPath, _1, _2));

    ConfigurationSpace CSforGoal(7); 
    CSforGoal.bounds(0,  0.26, 0.705); // body z
    CSforGoal.bounds(1, -0.5, 0.5); // body roll
    CSforGoal.bounds(2, -0.0, 0.5); // body pitch
    CSforGoal.bounds(3, -0.5, 0.5); // body yaw
    CSforGoal.bounds(4, -M_PI/2, M_PI/2);   // hand roll
    CSforGoal.bounds(5, -M_PI/2, M_PI/2); // hand pitch
    CSforGoal.bounds(6, -M_PI, M_PI);   // hand yaw

    planner->setConfiguration(startCfg);
    prob.updateOLV();

    struct timeval tv1, tv2;
    double Psample = 0.1;
    bool ret = false;
    int n;
    std::vector<reachingRRT> rrts;
    rrts.push_back(reachingRRT(rrt));
    std::vector<subproblem *> subproblems;
    int subproblemId=-1;
    Configuration intermediateGoal(CSforGoal.size());
    Configuration goalForPP(CSforPath->size());
    std::vector<Configuration> reachingPath;
    std::vector<Configuration> manipulationPath;
    gettimeofday(&tv1, NULL);
    for (n=0;;n++){
        gettimeofday(&tv2, NULL);
        if (tv2.tv_sec - tv1.tv_sec > 10) break;
        // reaching planning
        for (size_t i=0; i<rrts.size(); i++){
            reachingRRT &rrrt = rrts[i];
            RRT *rrt = rrrt.rrt;

            RoadmapPtr Tg  = rrt->getBackwardTree();
            if (!Tg->nNodes() || rrrt.isSolved || rand() < Psample*RAND_MAX){
                // try to generate a goal
                if (testRandomConfig(planner, setterForIntermediateGoal, 
                                     CSforGoal, intermediateGoal)){
                    //std::cout << "found an intermediate goal:" << intermediateGoal << std::endl;
                    for (int i=0; i<4; i++) goalForPP[i] = intermediateGoal[i];
                    for (int i=0; i<2; i++){
                        for (int j=0; j<armPath[i]->numJoints(); j++){
                            goalForPP[4+i*6+j] = armPath[i]->joint(j)->q;
                        }
                    }
                    RoadmapNodePtr node = RoadmapNodePtr(new RoadmapNode(goalForPP));
                    if (Tg->nNodes()){
                        RRT *rrt_new = new RRT(planner);
                        rrt_new->extendFromGoal(true);
                        rrt_new->epsilon(0.1);
                        rrt_new->setForwardTree(rrt->getForwardTree());
                        rrt_new->getBackwardTree()->addNode(node);
                        rrts.push_back(reachingRRT(rrt_new));
                    }else{
                        Tg->addNode(node);
                    }
                    subproblems.push_back(
                        new subproblem(&prob, &cd, robot,
                                       intermediateGoal, 
                                       setterForIntermediateGoal.reachedArm(),
                                       goalP, angle));
                    //prob.updateOLV();
                    //exit(0);
                }
            }else{
                // extend tree
                if (rrt->extendOneStep()){
                    //std::cout << "found a reaching path for " << i << "th goal" << std::endl;
                    rrrt.isSolved = true;
                    rrt->extractPath(rrrt.path);
                    if (subproblems[i]->isSolved){
                        ret = true;
                        subproblemId = i;
                        break;
                    }
                }
            }
        }
        // manipulation planning
        for (size_t i=0; i<subproblems.size(); i++){
            if (subproblems[i]->isSolved) continue;
            if (subproblems[i]->oneStep()) {
                if (rrts[i].isSolved) {
                    ret = true;
                    subproblemId = i;
                    break;
                }
            }
        }
        if (ret) break;
    }
    std::cout << "subproblemId:" << subproblemId << std::endl;
    std::cout << "rrts.size() = " << rrts.size() << ", subproblems.size() = " << subproblems.size() << std::endl;
    if (ret){
        {
            reachingPath = rrts[subproblemId].path;
            RandomShortcutOptimizer opt1(planner);
            ShortcutOptimizer       opt2(planner);
            for (int i=0; i<5; i++) {
                reachingPath = opt1.optimize(reachingPath);
                reachingPath = opt2.optimize(reachingPath);
            }
        }
        {
            ((RRT *)subproblems[subproblemId]->planner->getAlgorithm())->extractPath(manipulationPath);
            RandomShortcutOptimizer opt1(subproblems[subproblemId]->planner);
            ShortcutOptimizer       opt2(subproblems[subproblemId]->planner);
            for (int j=0; j<5; j++) {
                manipulationPath = opt1.optimize(manipulationPath);
                manipulationPath = opt2.optimize(manipulationPath);
            }
        }
    }
    gettimeofday(&tv2, NULL);
    bool bInterpolate = true;
    //bool bInterpolate = false;
    prob.dt(0.1);
    if (ret){
        std::ofstream ofs("path.txt");
        ofs << subproblems[subproblemId]->usedArm << std::endl;
        std::cout << "length of the reaching path:" << reachingPath.size()
                  << std::endl;
        if (bInterpolate){
            for (unsigned int i=1; i<reachingPath.size(); i++){
                std::vector<Configuration> interpolatedReachingPath;
                //planner->getMobility()->interpolationDistance(1.0);
                planner->getMobility()->getPath(reachingPath[i-1],reachingPath[i],
                                                interpolatedReachingPath);
                for (size_t j=0; j<interpolatedReachingPath.size(); j++){
                    planner->setConfiguration(interpolatedReachingPath[j]);
                    ofs << robot << std::endl;
                    prob.updateOLV();
                }
            }
        }else{
            for (unsigned int i=0; i<reachingPath.size(); i++){
                planner->setConfiguration(reachingPath[i]);
                ofs << robot << std::endl;
                prob.updateOLV();
            }
        }
        std::cout << "length of the manipulation path:" 
                  << manipulationPath.size() << std::endl;
        for (unsigned int i=1; i<manipulationPath.size(); i++){
            PathPlanner *planner = subproblems[subproblemId]->planner;
            if (bInterpolate){
                std::vector<Configuration> interpolatedManipulationPath;
                //planner->getMobility()->interpolationDistance(1.0);
                planner->getMobility()->getPath(
                    manipulationPath[i-1], manipulationPath[i],
                    interpolatedManipulationPath);
                for (size_t j=1; j<interpolatedManipulationPath.size(); j++){
                    planner->setConfiguration(interpolatedManipulationPath[j]);
                    ofs << robot << std::endl;
                    prob.updateOLV();
                }
            }else{
                planner->setConfiguration(manipulationPath[i]);
                ofs << robot << std::endl;
                prob.updateOLV();
            }
        }
        double time = (tv2.tv_sec - tv1.tv_sec) + ((double)(tv2.tv_usec - tv1.tv_usec))/1e6;
        std::cout << "total time = " << time << "[s]" << std::endl;
        double cdtime = planner->timeCollisionCheck();
        std::cout << "time spent in collision detection:"
                  << cdtime << "[s](" << cdtime/time*100 << "[%])" << std::endl;
        std::cout << "collision is checked " << planner->countCollisionCheck() << "times(" << cdtime/planner->countCollisionCheck()*1000 << "[ms/call])" << std::endl;
    }else{
        std::cout << "failed to find a path" << std::endl;
        std::cout << "profile of setter for intermediate goal:";
        setterForIntermediateGoal.profile();
        std::cout << "nodes from start:" << rrt->getForwardTree()->nNodes()
                  << std::endl;
        for (size_t i=0; i<subproblems.size(); i++){
            std::cout << i << " : nnodes = "
                      << rrts[i].rrt->getBackwardTree()->nNodes() << "("
                      << rrts[i].isSolved << "),";
            PathPlanner *p = subproblems[i]->planner;
            RRT *rrt = (RRT *)p->getAlgorithm();
            std ::cout << rrt->getForwardTree()->nNodes() << ","
                       << rrt->getBackwardTree()->nNodes() << ", profile=";
            subproblems[i]->setterForGoal.profile();
        }
#if 0
        std::cout << "nnodes = " << rrt->getForwardTree()->nNodes() << ","
                  << Tg->nNodes() << std::endl;
#endif
    }
    std::cout << rrts.size() << " RRTs are used to find a reaching motion"
              << std::endl;
    std::cout << "loop count = " << n << std::endl;

    return 0;
}
