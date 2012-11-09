#include <fstream>
#include <hrpUtil/TimeMeasure.h>
#include <hrpModel/Link.h>
#include <Math/Physics.h>
#include <Solver/DistanceConstraint.h>
#include "SphereTreeUtil.h"
#include "SphereTree.h"
#include "problem.h"
#include "Filter.h"
#include "ExecUtil.h"

using namespace motion_generator;
using namespace motion_interpolator;
using namespace hrp;

int main(int argc, char *argv[])
{
    const char *robotURL = NULL;
    bool avoidCollision=true, keepBalance=true, display=true;
    const char *cloudf=NULL;
    const char *pathf="path.txt"; 
    std::vector<std::string> obstacleURL;
    std::vector<Vector3> obstacleP;
    std::vector<Vector3> obstacleRpy;
    for (int i=0; i<argc; i++){
        if (strcmp(argv[i], "-robot") == 0){
            robotURL = argv[++i];
        }else if (strcmp(argv[i], "-no-balance-compensation")==0){
            keepBalance = false;
        }else if (strcmp(argv[i], "-no-collision-avoidance")==0){
            avoidCollision = false;
        }else if (strcmp(argv[i], "-no-display")==0){
            display = false;
        }else if (strcmp(argv[i], "-point-cloud")==0){
            cloudf = argv[++i];
        }else if (strcmp(argv[i], "-path")==0){
            pathf = argv[++i];
        }else if (strcmp(argv[i], "-obstacle") == 0){
            obstacleURL.push_back(argv[++i]);
            Vector3 p, rpy;
            for (int j=0; j<3; j++) p[j] = atof(argv[++i]);
            obstacleP.push_back(p);
            for (int j=0; j<3; j++) rpy[j] = atof(argv[++i]);
            obstacleRpy.push_back(rpy);
        }
    }
    std::cout << "collision avoidance = " << avoidCollision
              << ", balance compensation = " << keepBalance 
              << ", display = " << display << std::endl;

    if (robotURL == NULL){
        std::cerr << "please specify URL of VRML model by -robot option"
                  << std::endl;
        return 1;
    }
    // load robot
    HumanoidBodyPtr robot = HumanoidBodyPtr(new HumanoidBody());
    loadHumanoidBodyFromModelLoader(robot, robotURL, argc, argv, false);
    
    // load path
    int arm;
    std::vector<hrp::dvector> qPath;
    std::vector<hrp::Vector3> pPath;
    std::vector<hrp::Vector3> rpyPath;
    loadPath(pathf, robot->numJoints(), arm, qPath, pPath, rpyPath);
    
    // load shapes
    std::vector<DistanceGeometry *> shapes;
    loadShape("hrp2.shape", robot, shapes);

    Filter *filter = new Filter(robot);
    filter->selectArm(arm);
    filter->considerBalance(keepBalance);
    filter->init(qPath[0], pPath[0], rotFromRpy(rpyPath[0]));
    filter->setRobotShapes(shapes);

    setupSelfCollisionCheckPairs("hrp2.pairs", shapes, filter);

    dvInterpolator *qTrj = new dvInterpolator(robot->numJoints());
    v3Interpolator *pTrj = new v3Interpolator(3);
    v3Interpolator *rpyTrj = new v3Interpolator(3);
    int totalFrames 
        = setupInterpolator(qPath, pPath, rpyPath, qTrj, pTrj, rpyTrj);
    filter->duration(totalFrames*DT);

    // viewer
    problem prob(0);
    std::vector<BodyPtr> obstacles;
    if (1/*display*/){
        prob.addRobot("robot", robotURL, robot);
        for (unsigned int i=0; i<obstacleURL.size(); i++){
            char buf[20];
            sprintf(buf, "obstacle%02d", i);
            BodyPtr obstacle = prob.addObstacle(buf, obstacleURL[i]);
            Link *root = obstacle->rootLink();
            root->p = obstacleP[i];
            root->R = hrp::rotFromRpy(obstacleRpy[i]);
            //root->setSegmentAttitude(hrp::rotFromRpy(obstacleRpy[i]));
            obstacles.push_back(obstacle);
        }
        prob.initOLV(argc, argv);
    }
    // sphere tree of environment
    std::vector<hrp::Vector3> points;
    if (cloudf) loadPointArray(cloudf, points);
    if (obstacleP.size() >= 2) addObstaclePoints(points, obstacleP[1]);
    SphereTree stree(obstacles[0]->rootLink(), points, 0.01);
    stree.updatePosition();
    
#if 0 // confirm collision-free
    for (unsigned int i=0; i<qPath.size(); i++){
        robot->setPosture(qPath[i], pPath[i], rotFromRpy(rpyPath[i]));
        robot->calcForwardKinematics();
        std::vector<DistanceConstraint *> consts;
        createDistanceConstraints(stree, shapes, consts);
        std::cout << "nconsts:" << consts.size() << std::endl;
        for (unsigned int j=0; j<consts.size(); j++){
            consts[j]->computeDistance();
            if (consts[j]->distance() > stree.radius()) std::cout << consts[j]->distance() << " "; 
        }
        std::cout << std::endl;
    }
    exit(0);
#endif

#if 0 // show initial path
    for (unsigned int i=0; i<qPath.size(); i++){
        robot->setPosture(qPath[i], pPath[i], rotFromRpy(rpyPath[i]));
        robot->calcForwardKinematics();
        if (display) prob.updateOLV();
    }
    robot->setPosture(qPath[0], pPath[0], rotFromRpy(rpyPath[0]));
    robot->calcForwardKinematics();
#endif

    double tm = 0;
    std::ofstream comf("com.dat");
    std::ofstream qf("pos.dat");
    std::ofstream tmf("tm.dat");
    std::ofstream constf("const.dat");
    hrp::dvector q(robot->numJoints());
    hrp::Vector3 p, rpy;
    std::vector<DistanceConstraint *> consts;
    TimeMeasure disttm, qptm;
    int frames = 0;
    int extraFrames = 1.0/DT;
    while (frames < totalFrames+extraFrames){
        fprintf(stderr, "%6.3f/%6.3f\r", tm, totalFrames*DT);
        // compute reference motion
        int pos = frames >= totalFrames ? totalFrames-1 : frames;
        qTrj->get(pos, q);
        pTrj->get(pos, p);
        rpyTrj->get(pos, rpy);
        frames++;

        disttm.begin();
        createDistanceConstraints(stree, shapes, consts);
        disttm.end();

        if (avoidCollision) filter->setDistanceConstraints(consts);

        // compute feasible motion
        qptm.begin();
        bool ret = filter->filter(q, p, rotFromRpy(rpy));
        qptm.end();
        
        hrp::Vector3 com = robot->calcCM();
        comf << tm << " " << com[0] << " " << com[1] << " " << com[2] << std::endl;
        qf << tm;
        for (int i=0; i<robot->numJoints(); i++){
            qf << " " << robot->joint(i)->q;
        }
        qf << std::endl;

        tmf << tm << " " << disttm.time() << " " << qptm.time() << std::endl; 

        double minD;
        for (unsigned int i=0; i<consts.size(); i++){
            hrp::Vector3 s, e;
            consts[i]->computeDistance();
            double d = consts[i]->distance();
            if (i==0 || d < minD) minD = d;
        }
        if (consts.size()){
            constf << tm << " " << consts.size() << " " << minD << std::endl;
        }else{
            constf << std::endl << std::endl;
        }
#if 0
        for (unsigned i=0; i<consts.size(); i++){
            std::cout << i << ":" << consts[i]->distance() << ","
                      << consts[i]->value()[0] << std::endl;
        } 
#endif

        if (display) {
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
        tm += DT;
        if (!ret) {
#if 0
            for (unsigned int i=0; i<consts.size(); i++){
                consts[i]->computeDistance();
                std::cout << "d=" << consts[i]->distance() << std::endl;
            }
#endif
            break;
        }
    }
    std::cout << "creating distance constraints(avg/max):"
              << disttm.averageTime()*1000 << "/" 
              << disttm.maxTime()*1000<< "[ms]" << std::endl;
    std::cout << "time for solving QP(avg/max):"
              << qptm.averageTime()*1000 << ","
              << qptm.maxTime()*1000 << "[ms]" << std::endl;
    
    std::ofstream ofs("initcfg.txt");
    for (int i=0; i<3; i++){
        ofs << robot->rootLink()->p[i] << " "; 
    }
    for (int i=0; i<3; i++){
        for (int j=0; j<3; j++){
            ofs << robot->rootLink()->R(i,j) << " ";
        }
    }
    for (int i=0; i<robot->numJoints(); i++){
        ofs << robot->joint(i)->q << " ";
    }
    ofs << std::endl;

    return 0;
}

