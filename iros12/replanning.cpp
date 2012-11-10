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
#include <Solver/DistanceConstraint.h>
#include <hrpModel/JointPath.h>
#include <hrpModel/Link.h>
#include <hrpPlanner/RRT.h>
#include <hrpPlanner/Roadmap.h>
#include <hrpPlanner/RoadmapNode.h>
#include <hrpPlanner/RandomShortcutOptimizer.h>
#include <hrpPlanner/ShortcutOptimizer.h>
#include <hrpPlanner/ConfigurationSpace.h>
#include <Math/Physics.h>
#include "problem.h"
#include "myCfgSetter2.h"
#include "myCfgSetter3.h"
#include "ExecUtil.h"
#include "SphereTreeUtil.h"
#include "SphereTree.h"
#include "Filter.h"
#include "CustomCD.h"
#include "RobotUtil.h"

#define SHAPE_FILE "hrp2.shape"
#define PAIR_FILE  "hrp2.pairs"

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

void extractCfg(HumanoidBodyPtr robot, JointPathPtr armPath[2], 
                Configuration& cfg)
{
    cfg[0] = robot->rootLink()->p[2];
    Vector3 rpy = rpyFromRot(robot->rootLink()->R);
    cfg[1] = rpy[0]; cfg[2] = rpy[1]; cfg[3] = rpy[2];
    for (int j=0; j<2; j++){
        for (int i=0; i<armPath[j]->numJoints(); i++){
            cfg[4+j*6+i] = armPath[j]->joint(i)->q;
        }
    }
}

int main(int argc, char *argv[])
{
    srand((unsigned)time(NULL));
 
    const char *robotURL = NULL;
    const char *goalURL = NULL;
    const char *initposf=NULL;
    std::vector<std::string> obstacleURL;
    std::vector<Vector3> obstacleP;
    std::vector<Vector3> obstacleRpy;
    Vector3 p, rpy;
    bool display = true;
    double timeout = 3.0;
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
        }else if (strcmp(argv[i], "-init-pos")==0){
            initposf = argv[++i];
        }else if (strcmp(argv[i], "-timeout")==0){
            timeout = atof(argv[++i]);
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
    if (initposf){
        std::ifstream ifs(initposf);
        if (!ifs.is_open()){
            std::cerr << "failed to open " << initposf << std::endl;
            return 1;
        }
        ifs >> robot;
        robot->calcForwardKinematics();
    }

    Matrix33 goalR(rotFromRpy(goalRpy));
    if (goal){
        goal->rootLink()->p = goalP;
        goal->rootLink()->R = goalR;
        goal->calcForwardKinematics();
    }

    myCfgSetter3 setterForGoal(robot, goalP);
    myCfgSetter2 setterForPath(robot);

    // sphere tree of environment
    const char *cloudf = "plant.pc";
    std::vector<hrp::Vector3> points;
    loadPointArray(cloudf, points);

    SphereTree stree(obstacles[0]->rootLink(), points, 0.01);
    stree.updatePosition();

    CustomCD cd(robot, SHAPE_FILE, PAIR_FILE, obstacles[0], &stree);
    cd.tolerance(0.005);
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

    Configuration startCfg(CSforPath->size()), goalCfg(CSforPath->size());
    startCfg[0] = robot->rootLink()->p[2];
    rpy = rpyFromRot(robot->rootLink()->R);
    startCfg[1] = rpy[0]; startCfg[2] = rpy[1]; startCfg[3] = rpy[2];
    for (int j=0; j<2; j++){
        for (int i=0; i<armPath[j]->numJoints(); i++){
            startCfg[4+j*6+i] = armPath[j]->joint(i)->q;
        }
    }
    planner->setApplyConfigFunc(boost::bind(&myCfgSetter2::set, 
                                            &setterForPath, _1, _2));

    planner->setConfiguration(startCfg);
    //prob.updateOLV();
    if (planner->checkCollision()){
        std::cerr << "Error:start configuration is not collision-free" << std::endl;
        return 1;
    }

    ConfigurationSpace CSforGoal(7); 
    CSforGoal.bounds(0,  0.26, 0.705); // body z
    CSforGoal.bounds(1, -0.5, 0.5); // body roll
    CSforGoal.bounds(2, -0.0, 0.5); // body pitch
    CSforGoal.bounds(3, -0.5, 0.5); // body yaw
    CSforGoal.bounds(4, -M_PI/2, M_PI/2);   // hand roll
    CSforGoal.bounds(5, -M_PI/2, M_PI/2); // hand pitch
    CSforGoal.bounds(6, -M_PI, M_PI);   // hand yaw
    for (int i=0; i<7; i++){
        CSforGoal.weight(i) = 1.0;
    }

    // load shapes
    std::vector<DistanceGeometry *> shapes;
    loadShape(SHAPE_FILE, robot, shapes);

    Filter *filter = new Filter(robot);
    filter->setRobotShapes(shapes);

    double Psample = 0.1;
    RoadmapPtr Tg  = rrt->getBackwardTree();
    TimeMeasure tm;
    int ntimeout=0;
    bool initialPlan=true;
    struct timeval tv1, tv2;
    //
    while(1){
        // plan
        std::cout << "planning" << std::endl;
        std::vector<RoadmapNodePtr> goals;
        std::vector<int> arms;
        rrt->getForwardTree()->clear();
        rrt->getBackwardTree()->clear();
        bool ret = false;
        int arm=-1;
        extractCfg(robot, armPath, startCfg);
        setterForGoal.resetCog();
        setterForPath.resetCog();
#if 1
        planner->setConfiguration(startCfg);
        //prob.updateOLV();
        if (planner->checkCollision()){
            std::cerr << "Error:start configuration is not collision-free" << std::endl;
            const std::vector<CdShape>& shapes = cd.shapes();
            for (unsigned int i=0; i<shapes.size(); i++){
#if 1
                double d;
                if (shapes[i].type() == CdShape::SPHERE){
                    d = stree.distance(shapes[i].center(), shapes[i].radius());
                }else{
                    d = stree.distance(shapes[i].center(0),
                                       shapes[i].center(1),
                                       shapes[i].radius());
                }
                if (d < 0){
                    std::cout << shapes[i].link()->name << " is colliding(" << d << ")" << std::endl;
                }
#else
                if (shapes[i].type() == CdShape::SPHERE){
                    std::cout << stree.isColliding(shapes[i].center(), shapes[i].radius()) << ","
                              << stree.distance(shapes[i].center(), shapes[i].radius()) << std::endl;
                }else{
                    std::cout << stree.isColliding(shapes[i].center(0),
                                                   shapes[i].center(1),
                                                   shapes[i].radius())
                              << ","
                              << stree.distance(shapes[i].center(0),
                                                shapes[i].center(1),
                                                shapes[i].radius())
                              << std::endl;
                }
#endif
            }

            return 1;
        }
#endif
        rrt->getForwardTree()->addNode(RoadmapNodePtr(new RoadmapNode(startCfg)));
        gettimeofday(&tv1, NULL);
        tm.begin();
        while(1){
            if (!Tg->nNodes() || rand() < Psample*RAND_MAX){
                if (find_a_goal(prob, setterForGoal, CSforGoal, goalCfg, armPath)){
                    std::cout << "found a goal" << std::endl;
                    //prob.updateOLV();
                    RoadmapNodePtr goal = RoadmapNodePtr(new RoadmapNode(goalCfg));
                    Tg->addNode(goal);
                    goals.push_back(goal);
                    arms.push_back(setterForGoal.reachedArm());
                }
            }else{
                if (ret = rrt->extendOneStep()) {
                    // found a path
                    RoadmapNodePtr node = Tg->lastAddedNode();
                    while (node->nChildren()) node = node->child(0);
                    for (size_t i=0; i<goals.size(); i++){
                        if (goals[i] == node){
                            arm = arms[i];
                            std::cout << "used arm is " << arm << std::endl;
                            break;
                        }
                    }
                    break;
                }
            }
            gettimeofday(&tv2, NULL);
            if (tv2.tv_sec - tv1.tv_sec + (tv2.tv_usec - tv1.tv_usec)/1e6
                > timeout) {
                ntimeout++;
                break;
            }
        }
        std::vector<Configuration> path;
        if (ret){
            // smoothing
            std::cout << "smoothing" << std::endl;
            rrt->extractPath(path);
            RandomShortcutOptimizer opt1(planner);
            ShortcutOptimizer       opt2(planner);
            for (int i=0; i<50; i++) {
                path = opt1.optimize(path);
                path = opt2.optimize(path);
            }
        }
        tm.end();
        if (ret){
            if (initialPlan && obstacleP.size()>=2){
                addObstaclePoints(points, obstacleP[1]);
                stree.build(points, 0.01);
                initialPlan = false;
            }
            std::ofstream ofs("path.txt");
            ofs << arm << std::endl;

            // extract the initial path
            std::vector<hrp::dvector> qPath;
            std::vector<hrp::Vector3> pPath;
            std::vector<hrp::Vector3> rpyPath;
            Link *root = robot->rootLink();
            dvector q;
            for (unsigned int i=0; i<path.size(); i++){
                planner->setConfiguration(path[i]);
                //prob.updateOLV();
                robot->getPosture(q);
                qPath.push_back(q);
                pPath.push_back(root->p);
                rpyPath.push_back(rpyFromRot(root->R));
                ofs << robot << std::endl;
            }
            // transform the path into a trajectory
            dvInterpolator *qTrj = new dvInterpolator(robot->numJoints());
            v3Interpolator *pTrj = new v3Interpolator(3);
            v3Interpolator *rpyTrj = new v3Interpolator(3);
            int totalFrames 
                = setupInterpolator(qPath, pPath, rpyPath, qTrj, pTrj, rpyTrj);
            // execute the trajectory
            std::cout << "executing" << std::endl;
            int extraFrames = 1.0/DT;
            int frames = 0;
            filter->selectArm(arm);
            filter->duration(totalFrames*DT);
            setupSelfCollisionCheckPairs(PAIR_FILE, shapes, filter);
            filter->init(qPath[0], pPath[0], rotFromRpy(rpyPath[0]));
            while (frames < totalFrames+extraFrames){
                //fprintf(stderr, "%6.3f/%6.3f\r", tm, totalFrames*DT);
                // compute reference motion
                int pos = frames >= totalFrames ? totalFrames-1 : frames;
                qTrj->get(pos, q);
                pTrj->get(pos, p);
                rpyTrj->get(pos, rpy);
                frames++;

                std::vector<DistanceConstraint *> consts;
                createDistanceConstraints(stree, shapes, consts);
                filter->setDistanceConstraints(consts);

                // compute feasible motion
                if (!filter->filter(q, p, rotFromRpy(rpy))) {
                    std::cerr << "execution failed" << std::endl;
                    break;
                }
                std::vector<hrp::Vector3> starts, ends;
                for (unsigned int i=0; i<consts.size(); i++){
                    hrp::Vector3 s, e;
                    consts[i]->computeDistance();
                    double d = consts[i]->distance();
                    consts[i]->closestPoints(s,e);
                    starts.push_back(s);
                    ends.push_back(e);
                }
                prob.updateOLV(starts, ends);
            }
            // check hand position
            Vector3 curP;
            robot->calcGraspPosition(arm, curP);
            Vector3 err = goalP - curP;
            std::cout << "err = " << err.norm() << std::endl;
            if (err.norm() < 0.01) break; // reached
        }else{
            std::cout << "failed to find a path" << std::endl;
            std::cout << "nnodes = " << rrt->getForwardTree()->nNodes() << ","
                      << Tg->nNodes() << std::endl;
            break;
        }
#if 0 // only once
        break;
#endif
    }

    return 0;
}
