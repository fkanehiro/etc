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
#include "problem.h"
#include "CustomCD.h"
#include "myCfgSetter2.h"
#include <Math/Physics.h>
#include "RobotUtil.h"

using namespace motion_generator;
using namespace hrp;


int main(int argc, char *argv[])
{
    srand((unsigned)time(NULL));
 
    const char *robotURL = NULL;
    std::vector<std::string> obstacleURL;
    std::vector<Vector3> obstacleP;
    std::vector<Vector3> obstacleRpy;
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

    // This must be called after all bodies are added
    prob.initOLV(argc, argv);

    PathEngine::ConfigurationSpace* cspace
        = prob.planner()->getConfigurationSpace();
    cspace->bounds(0,  0.26, 0.705); // body z
    cspace->bounds(1, -0.5, 0.5); // body roll
    cspace->bounds(2, -0.0, 0.5); // body pitch
    cspace->bounds(3, -0.5, 0.5); // body yaw

    int arm;
    PathEngine::Configuration goalCfg(cspace->size());
    std::ifstream ifs("goal.txt");
    ifs >> arm;
    for (unsigned int i=0; i<cspace->size(); i++){
        ifs >> goalCfg[i];
    }

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

    cspace->weight(10) = 0.8;
    cspace->weight(11) = 0.6;
    cspace->weight(12) = 0.4;
    cspace->weight(13) = 0.3;
    cspace->weight(14) = 0.2;
    cspace->weight(15) = 0.1;
#endif

    JointPathPtr armPath[2];
    for (int k=0; k<2; k++){
        armPath[k] = robot->getJointPath(robot->chestLink,
                                         robot->wristLink[k]);
        for (int i=0; i<armPath[k]->numJoints(); i++){
            Link *j = armPath[k]->joint(i);
            cspace->bounds(4+k*6+i, j->llimit, j->ulimit);
        }
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
    setHalfConf(robot);

    myCfgSetter2 setter(robot);

    PathEngine::Configuration startCfg(cspace->size());
    startCfg[0] = robot->rootLink()->p[2];
    startCfg[1] = startCfg[2] = startCfg[3] = 0;
    for (int j=0; j<2; j++){
        for (int i=0; i<armPath[j]->numJoints(); i++){
            startCfg[4+j*6+i] = armPath[j]->joint(i)->q;
        }
    }
    planner->setStartConfiguration(startCfg);
    planner->setGoalConfiguration(goalCfg);
    planner->setApplyConfigFunc(boost::bind(&myCfgSetter2::set, &setter,
                                            _1, _2));
    planner->setConfiguration(startCfg);
    prob.updateOLV();
    planner->setConfiguration(goalCfg);
    prob.updateOLV();
#if 0
    int earm;
    ifs >> earm;
    while (!ifs.eof()){
        PathEngine::Configuration cfg;
        for (unsigned int i=0; i<PathEngine::Configuration::size(); i++){
            ifs >> cfg[i];
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

    std::string shapeFile = robot->modelName()+".shape";
    CustomCD cd(robot, shapeFile.c_str(), "hrp2.pairs", 
                obstacles[0], "plant.pc");
    cd.tolerance(0.005);
    prob.planner()->setCollisionDetector(&cd);
    
    struct timeval tv1, tv2;
    gettimeofday(&tv1, NULL);
    // plan
    bool ret = planner->calcPath();
    if (ret){
        std::cout << "found a path. optimizing..." << std::flush;
        for (int i=0; i<5; i++) {
            planner->optimize("RandomShortcut");
            planner->optimize("Shortcut");
        }
        std::cout << "done." << std::endl;
    }
    gettimeofday(&tv2, NULL);
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
            prob.updateOLV();
        }
    }else{
        PathEngine::RoadmapPtr roadmap = planner->getRoadmap();
        for (unsigned int i=0; i<roadmap->nNodes(); i++){
            PathEngine::RoadmapNodePtr node = roadmap->node(i);
            planner->setConfiguration(node->position());
            prob.updateOLV();
        }
    }
    double time = (tv2.tv_sec - tv1.tv_sec) + ((double)(tv2.tv_usec - tv1.tv_usec))/1e6;
    std::cout << "total time = " << time << "[s]" << std::endl;
    double cdtime = prob.planner()->timeCollisionCheck();
    std::cout << "time spent in collision detection:"
              << cdtime << "[s](" << cdtime/time*100 << "[%]), "
              << cdtime*1e3/prob.planner()->countCollisionCheck() << "[ms/call]" << std::endl;
    std::cout << "profile:";
    setter.profile();

    return 0;
}
