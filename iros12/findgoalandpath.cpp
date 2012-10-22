/*
  把持動作の探索
  把持の位置は点で、回転は範囲で与えられる
  環境は球の階層で、ロボットは球またはカプセルで近似した形状が干渉チェックに
  使用される
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
#include "myCfgSetter2.h"
#include "myCfgSetter3.h"
#include <Math/Physics.h>
#include "CustomCD.h"

using namespace motion_generator;
using namespace hrp;
using namespace PathEngine;

bool find_a_goal(problem &prob, myCfgSetter3 &setter, 
                 ConfigurationSpace& cspace, 
                 Configuration& goalCfg, JointPathPtr armPath[2])
{
    Configuration cfg = cspace.random();
    if (setter.set(prob.planner(), cfg) && !prob.planner()->checkCollision()){
        for (int i=0; i<4; i++) goalCfg[i] = cfg[i];
        for (int i=0; i<2; i++){
            for (int j=0; j<armPath[i]->numJoints(); j++){
                goalCfg[4+i*6+j] = armPath[i]->joint(j)->q;
            }
        }
        return true;
    }
    return false;
}

int main(int argc, char *argv[])
{
    srand((unsigned)time(NULL));
 
    const char *robotURL = NULL;
    const char *goalURL = NULL;
    std::vector<std::string> obstacleURL;
    std::vector<Vector3> obstacleP;
    std::vector<Vector3> obstacleRpy;
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
    robot->rootLink()->p(2) = waistHeight;
    for (int i=0; i<robot->numJoints(); i++){
        robot->joint(i)->q = halfconf[i];
    }
    robot->calcForwardKinematics();

    Matrix33 goalR(rotFromRpy(goalRpy));
    if (goal){
        goal->rootLink()->p = goalP;
        goal->rootLink()->R = goalR;
        goal->calcForwardKinematics();
    }

    myCfgSetter3 setterForGoal(robot, goalP);
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

    CSforPath->bounds(0,  0.2, 0.8); // body z
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

    Configuration startCfg(CSforPath->size()), goalCfg(CSforPath->size());
    startCfg[0] = robot->rootLink()->p[2];
    startCfg[1] = startCfg[2] = startCfg[3] = 0;
    for (int j=0; j<2; j++){
        for (int i=0; i<armPath[j]->numJoints(); i++){
            startCfg[4+j*6+i] = armPath[j]->joint(i)->q;
        }
    }
    rrt->getForwardTree()->addNode(new RoadmapNode(startCfg));
    planner->setApplyConfigFunc(boost::bind(&myCfgSetter2::set, 
                                            &setterForPath, _1, _2));

    ConfigurationSpace CSforGoal(7); 
    CSforGoal.bounds(0,  0.2, 0.8); // body z
    CSforGoal.bounds(1, -0.5, 0.5); // body roll
    CSforGoal.bounds(2, -0.0, 0.5); // body pitch
    CSforGoal.bounds(3, -0.5, 0.5); // body yaw
    CSforGoal.bounds(4, -M_PI/2, M_PI/2);   // hand roll
    CSforGoal.bounds(5, -M_PI/2, M_PI/2); // hand pitch
    CSforGoal.bounds(6, -M_PI, M_PI);   // hand yaw
    for (int i=0; i<7; i++){
        CSforGoal.weight(i) = 1.0;
    }

    struct timeval tv1, tv2;
    Roadmap *Tg  = rrt->getBackwardTree();
    double Psample = 0.1;
    bool ret = false;
    int n=100000;
    gettimeofday(&tv1, NULL);
    for (int i=0; i<n; i++){
        if (!Tg->nNodes() || rand() < Psample*RAND_MAX){
            if (find_a_goal(prob, setterForGoal, CSforGoal, goalCfg, armPath)){
                Tg->addNode(new RoadmapNode(goalCfg));
            }
        }else{
            if (ret = rrt->extendOneStep()) break;
        }
    }
    std::vector<Configuration> path;
    if (ret){
        rrt->extractPath(path);
        RandomShortcutOptimizer opt1(planner);
        ShortcutOptimizer       opt2(planner);
        for (int i=0; i<5; i++) {
            path = opt1.optimize(path);
            path = opt2.optimize(path);
        }
    }
    gettimeofday(&tv2, NULL);
    if (ret){
        for (unsigned int i=0; i<path.size(); i++){
            planner->setConfiguration(path[i]);
            prob.updateOLV();
        }
        double time = (tv2.tv_sec - tv1.tv_sec) + ((double)(tv2.tv_usec - tv1.tv_usec))/1e6;
        std::cout << "total time = " << time << "[s]" << std::endl;
        double cdtime = planner->timeCollisionCheck();
        std::cout << "time spent in collision detection:"
                  << cdtime << "[s](" << cdtime/time*100 << "[%])" << std::endl;
        std::cout << "collision is checked " << planner->countCollisionCheck() << "times(" << cdtime/planner->countCollisionCheck()*1000 << "[ms/call])" << std::endl;

    }else{
        std::cout << "failed to find a path" << std::endl;
        std::cout << "profile of setter for goal:" << std::endl;
        setterForGoal.profile();
        std::cout << "nnodes = " << rrt->getForwardTree()->nNodes() << ","
                  << Tg->nNodes() << std::endl;
    }

    return 0;
}
