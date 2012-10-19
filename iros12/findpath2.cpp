/*
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
#include <hrpPlanner/ConfigurationSpace.h>
#include <hrpUtil/TimeMeasure.h>
#include "problem.h"
#include "CustomCD.h"
#include "myCfgSetter.h"
#include <Math/Physics.h>

using namespace motion_generator;
using namespace hrp;

int readGoal(std::ifstream &ifs, PathEngine::Configuration &goalCfg)
{
    int arm;
    double dummy;
    ifs >> arm;
    for (int i=0; i<4; i++) ifs >> goalCfg[i];
    if (arm == 0){
        for (int i=0; i<6; i++) ifs >> goalCfg[4+i];
        for (int i=0; i<6; i++) ifs >> dummy;
    }else{
        for (int i=0; i<6; i++) ifs >> dummy;
        for (int i=0; i<6; i++) ifs >> goalCfg[4+i];
    }
    return arm;
}

int main(int argc, char *argv[])
{
    srand((unsigned)time(NULL));
 
    const char *robotURL = NULL;
    std::vector<std::string> obstacleURL;
    std::vector<Vector3> obstacleP;
    std::vector<Vector3> obstacleRpy;
    bool display = true;
    int ngoal = 1;
    for(int i = 1 ; i < argc; i++){
        if (strcmp(argv[i], "-robot") == 0){
            robotURL = argv[++i];
        }else if (strcmp(argv[i], "-obstacle") == 0){
            obstacleURL.push_back(argv[++i]);
            Vector3 p, rpy;
            for (int j=0; j<3; j++) p[j] = atof(argv[++i]);
            obstacleP.push_back(p);
            for (int j=0; j<3; j++) rpy[j] = atof(argv[++i]);
            obstacleRpy.push_back(rpy);
        }else if (strcmp(argv[i], "-ngoal") == 0){
            ngoal = atoi(argv[++i]);
        }else if (strcmp(argv[i], "-no-display")==0){
            display = false;
        }
    }
    if (robotURL == NULL){
        std::cerr << "please specify URL of VRML model by -robot option"
                  << std::endl;
        return 1;
    }


    HumanoidBodyPtr robot = HumanoidBodyPtr(new HumanoidBody());
    loadHumanoidBodyFromModelLoader(robot, robotURL, argc, argv, true);

    problem prob(4+6);
    prob.addRobot("robot", robotURL, robot);
    std::vector<BodyPtr> obstacles;
    for (unsigned int i=0; i<obstacleURL.size(); i++){
        char buf[20];
        sprintf(buf, "obstacle%02d", i);
        obstacles.push_back(prob.addObstacle(buf, obstacleURL[i]));
    }

    // This must be called after all bodies are added
    if (display) prob.initOLV(argc, argv);

    PathEngine::ConfigurationSpace* cspace
        = prob.planner()->getConfigurationSpace();
    cspace->bounds(0,  0.2, 0.8); // body z
    cspace->bounds(1, -0.5, 0.5); // body roll
    cspace->bounds(2, -0.0, 0.5); // body pitch
    cspace->bounds(3, -0.5, 0.5); // body yaw

    PathEngine::Configuration goalCfg(cspace->size());
    std::ifstream ifs("goal.txt");

#if 1
    cspace->weight(0) = 0.1; // z

    cspace->weight(1) = 1;  // roll
    cspace->weight(2) = 1;  // pitch
    cspace->weight(3) = 1;  // yaw

    cspace->weight(4) = 0.8;
    cspace->weight(5) = 0.6;
    cspace->weight(6) = 0.4;
    cspace->weight(7) = 0.3;
    cspace->weight(8) = 0.2;
    cspace->weight(9) = 0.1;
#endif

    JointPathPtr armPath[2];
    for (int i=0; i<2; i++){
        armPath[i] = robot->getJointPath(robot->chestLink,
                                         robot->wristLink[i]);
    }

    PathEngine::PathPlanner *planner = prob.planner();
    planner->setMobilityName("OmniWheel");
    planner->setAlgorithmName("RRT");
    PathEngine::RRT *rrt = (PathEngine::RRT *)planner->getAlgorithm();
    rrt->extendFromGoal(true);
    planner->getAlgorithm()->setProperty("eps", "0.1");
    planner->getAlgorithm()->setProperty("max-trials", "50000");
    planner->getAlgorithm()->setProperty("interpolation-distance", "0.05");
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

    myCfgSetter setter[] = {myCfgSetter(robot, 0),
                            myCfgSetter(robot, 1)};

    PathEngine::Configuration startCfg(cspace->size());
    startCfg[0] = robot->rootLink()->p[2];
    startCfg[1] = startCfg[2] = startCfg[3] = 0;

#if 0
    int earm;
    ifs >> earm;
    while (!ifs.eof()){
        PathEngine::Configuration cfg(cspace->size());
        for (int i=0; i<4; i++) ifs >> goalCfg[i];
        if (arm == 0){
            for (int i=0; i<6; i++) ifs >> goalCfg[4+i];
            for (int i=0; i<6; i++) ifs >> dummy;
        }else{
            for (int i=0; i<6; i++) ifs >> dummy;
            for (int i=0; i<6; i++) ifs >> goalCfg[4+i];
        }

        if (arm == earm) {
            rrt->addExtraGoal(cfg);
            std::cout << "added an extra goal" << std::endl;
            planner->setConfiguration(cfg);
            prob.updateOLV();
        }
        ifs >> earm;
    }
#endif

    CustomCD cd(robot, "hrp2.shape", "hrp2.pairs", 
                obstacles[0], "plant.pc");
    prob.planner()->setCollisionDetector(&cd);
    
    bool ret;
    TimeMeasure tm[2];
    for (int i=0; i<ngoal;i++){
        planner->getRoadmap()->clear();
        int arm = readGoal(ifs, goalCfg);
        for (int i=0; i<armPath[arm]->numJoints(); i++){
            Link *j = armPath[arm]->joint(i);
            cspace->bounds(4+i, j->llimit, j->ulimit);
        }
        robot->setPosture(halfconf);
        for (int i=0; i<armPath[arm]->numJoints(); i++){
            startCfg[4+i] = armPath[arm]->joint(i)->q;
        }
        planner->setStartConfiguration(startCfg);
        planner->setGoalConfiguration(goalCfg);
        planner->setApplyConfigFunc(boost::bind(&myCfgSetter::set, &setter[arm],
                                                _1, _2));

        if (display){
            planner->setConfiguration(startCfg);
            prob.updateOLV();
            planner->setConfiguration(goalCfg);
            prob.updateOLV();
        }

        // plan
        tm[arm].begin();
        ret = planner->calcPath();
        if (ret){
            for (int i=0; i<500; i++) {
                planner->optimize("RandomShortcut");
                planner->optimize("Shortcut");
            }
        }
        tm[arm].end();

        if (ret){
            const std::vector<PathEngine::Configuration>& postures 
                = planner->getWayPoints();
            std::ofstream ofs("path.txt");
            ofs << arm << std::endl;
            for (unsigned int i=0; i<postures.size(); i++){
                planner->setConfiguration(postures[i]);
                for (int j=0; j<3; j++){
                    ofs << robot->rootLink()->p[j] << " ";
                }
                for (int j=0; j<3; j++){
                    for (int k=0; k<3; k++){
                        ofs << robot->rootLink()->R(j,k) << " ";
                    }
                }
                for (int j=0; j<robot->numJoints(); j++){
                    ofs << robot->joint(j)->q << " ";
                }
                ofs << std::endl;
                if (display) prob.updateOLV();
            }
        }else{
            PathEngine::Roadmap *roadmap = planner->getRoadmap();
            for (unsigned int i=0; i<roadmap->nNodes(); i++){
                PathEngine::RoadmapNode *node = roadmap->node(i);
                planner->setConfiguration(node->position());
                if (display) prob.updateOLV();
            }
        }

    }
    std::cout << "goals(right/left) = " <<  tm[0].numCalls() << "/" << tm[1].numCalls() << std::endl; 
    double time = tm[0].totalTime() + tm[1].totalTime();
    std::cout << "total time = " << time << "[s](" 
              << time/ngoal*1000 << "[ms/goal])" << std::endl;
    for (int i=0; i<2; i++){
        std::cout << i << ": total time = " << tm[i].totalTime() << "[s](avg:" 
                  << tm[i].averageTime()*1000 << "[ms/goal], max:"
                  << tm[i].maxTime()*1000 << "[ms])" << std::endl;
    }
    double cdtime = prob.planner()->timeCollisionCheck();
    std::cout << "time spent in collision detection:"
              << cdtime << "[s](" << cdtime/time*100 << "[%])" << std::endl;
    std::cout << "collision is checked " << prob.planner()->countCollisionCheck() << "times(" << cdtime/prob.planner()->countCollisionCheck()*1000 << "[ms/call])" << std::endl;
    std::cout << "profile:" << std::endl;
    setter[0].profile();
    setter[1].profile();

    return 0;
}
