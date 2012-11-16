/*
  把持姿勢の探索
  把持の位置は点で、回転は範囲で与えられる
  環境は球の階層で、ロボットは球またはカプセルで近似した形状が干渉チェックに
  使用される
  探索した結果はgoal.txtに保存される。各行が1つの把持姿勢に対応し、順に
  …使用した腕
  …体幹の高さ、回転
　…腕の関節角度
　が記録される。
 */
#include <fstream>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <Model/HumanoidBodyUtil.h>
#include <hrpModel/JointPath.h>
#include <hrpModel/Link.h>
#include <hrpPlanner/ConfigurationSpace.h>
#include "problem.h"
#include "myCfgSetter3.h"
#include <Math/Physics.h>
#include "CustomCD.h"
#include "RobotUtil.h"

using namespace motion_generator;
using namespace hrp;


int main(int argc, char *argv[])
{
    srand((unsigned)time(NULL));
 
    const char *robotURL = NULL;
    const char *goalURL = NULL;
    std::vector<std::string> obstacleURL;
    std::vector<Vector3> obstacleP;
    std::vector<Vector3> obstacleRpy;
    Vector3 p, rpy;
    int ngoal=10;
    bool display = true, silent=false;
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
        }else if (strcmp(argv[i], "-ngoal") == 0){
            ngoal = atoi(argv[++i]);
        }else if (strcmp(argv[i], "-no-display")==0){
            display = false;
        }else if (strcmp(argv[i], "-silent")==0){
            silent = true;
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

    problem prob(0);
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

    JointPathPtr jp = robot->getJointPath(robot->chestLink, robot->wristLink[RIGHT]);
    int dim = jp->numJoints() == 7 ? 8 : 7;
    PathEngine::ConfigurationSpace cspace(dim); 
    cspace.bounds(0,  0.26, 0.705); // body z
    cspace.bounds(1, -0.5, 0.5); // body roll
    cspace.bounds(2, -0.0, 0.5); // body pitch
    cspace.bounds(3, -0.5, 0.5); // body yaw
    cspace.bounds(4, -M_PI/2, M_PI/2);   // hand roll
    cspace.bounds(5, -M_PI/2, M_PI/2); // hand pitch
    cspace.bounds(6, -M_PI, M_PI);   // hand yaw
    if (dim == 8){
        cspace.bounds(7,
                      robot->wristLink[RIGHT]->llimit,
                      robot->wristLink[RIGHT]->ulimit);
    }

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

    myCfgSetter3 setter(robot, goalP);

    CustomCD cd(robot, "hrp2.shape", "hrp2.pairs", 
                obstacles[0], "plant.pc");
    prob.planner()->setCollisionDetector(&cd);
    
    if (display) prob.updateOLV();

    Vector3 initialcom = robot->calcCM();
    struct timeval tv1, tv2;
    std::ofstream goalf("goal.txt");
    std::ofstream goalcfgf("goalcfg.txt");
    std::ofstream goalcmf("goalcm.txt");
    gettimeofday(&tv1, NULL);
    int n=0, c = 0;
    while (c < ngoal){
        PathEngine::Configuration cfg = cspace.random();
        if (setter.set(prob.planner(), cfg) && !prob.planner()->checkCollision()){
            c++;
            if (silent) continue;
            int arm = setter.reachedArm();
            goalf << arm << " ";
            for (unsigned int i=0; i<4; i++) goalf << cfg[i] << " "; 
            for (int j=0; j<2; j++){
                JointPathPtr armPath = robot->getJointPath(robot->chestLink,
                                                           robot->wristLink[j]);
                for (int i=0; i<armPath->numJoints(); i++) {
                    goalf << armPath->joint(i)->q << " "; 
                }
            }
            goalf << std::endl;
            goalcfgf << arm << " ";
            for (int i=0; i<7; i++){
                goalcfgf << cfg[i] << " ";
            }
            goalcfgf << std::endl;

            Vector3 com = robot->calcCM();
            goalcmf << com[0]-initialcom[0] << " " 
                    << com[1]-initialcom[1] << std::endl;
            if (display) prob.updateOLV();
        }
        if (!silent) fprintf(stderr, "%3d/%3d\r", c, ++n);
    }
    gettimeofday(&tv2, NULL);
    std::cout << std::endl;
    double time = (tv2.tv_sec - tv1.tv_sec) + ((double)(tv2.tv_usec - tv1.tv_usec))/1e6;
    std::cout << "total time = " << time << "[s]("
              << time/ngoal*1000 << "[ms/goal])" << std::endl;
    double cdtime = prob.planner()->timeCollisionCheck();
    std::cout << "time spent in collision detection:"
              << cdtime << "[s](" << cdtime/time*100 << "[%])" << std::endl;
    std::cout << "collision is checked " << prob.planner()->countCollisionCheck() << "times(" << cdtime/prob.planner()->countCollisionCheck()*1000 << "[ms/call])" << std::endl;
    std::cout << "profile:";
    setter.profile();

    return 0;
}
