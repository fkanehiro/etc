/*
 */
#include <fstream>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <Model/HumanoidBodyUtil.h>
#include <hrpModel/JointPath.h>
#include <hrpModel/Link.h>
#include "problem.h"
#include "CustomCD.h"
#include "myCfgSetter4.h"
#include <Math/Physics.h>

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


    HumanoidBodyPtr robot = new HumanoidBody();
    loadHumanoidBodyFromModelLoader(robot, robotURL, argc, argv, true);

    problem prob;
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
    prob.initOLV(argc, argv);

    PathEngine::Configuration::size(6); 
    PathEngine::Configuration::bounds(0,  0.2, 0.8); // body z
    PathEngine::Configuration::bounds(1, -0.5, 0.5); // body roll
    PathEngine::Configuration::bounds(2, -0.0, M_PI/4); // body pitch
    PathEngine::Configuration::bounds(3, -M_PI/4, M_PI/4); // body yaw
    PathEngine::Configuration::bounds(4, -0.05, 0.05);   // hand z
    //PathEngine::Configuration::bounds(5, -M_PI/2, M_PI/2); // hand pitch
    PathEngine::Configuration::bounds(5, -M_PI/2, M_PI/2); // hand pitch

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
    halfconf[2] = halfconf[ 8] = ToRad(-24);
    halfconf[3] = halfconf[ 9] = ToRad( 50);
    halfconf[4] = halfconf[10] = ToRad(-26);
    double leg_link_len1=0, leg_link_len2=0; 
    halfconf[16] = halfconf[23] = ToRad(10);
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

    myCfgSetter setter = myCfgSetter(robot, goalP);

    CustomCD cd(robot, "hrp2.shape", "hrp2.pairs", 
                obstacles[0], "table.pc");
    prob.planner()->setCollisionDetector(&cd);
    
    prob.updateOLV();

    Vector3 initialcom = robot->calcCM();
    struct timeval tv1, tv2;
    std::ofstream goalf("goal.txt");
    std::ofstream goalcmf("goalcm.txt");
    gettimeofday(&tv1, NULL);
    int n=0, c = 0;
    while (c < ngoal){
        PathEngine::Configuration cfg = PathEngine::Configuration::random();
        if (setter.set(prob.planner(), cfg) && !prob.planner()->checkCollision()){
            c++;
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

            Vector3 com = robot->calcCM();
            goalcmf << com[0]-initialcom[0] << " " 
                    << com[1]-initialcom[1] << std::endl;
            prob.updateOLV();
        }
        printf("%3d/%3d\r", c, ++n);
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
