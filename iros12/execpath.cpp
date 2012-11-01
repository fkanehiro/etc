#include <hrpUtil/TimeMeasure.h>
#include <Model/HumanoidBodyUtil.h>
#include <fstream>
#include <Solver/Sphere.h>
#include <Solver/CappedCylinder.h>
#include <Solver/CappedCylinderSpherePair.h>
#include <Solver/DistanceConstraint.h>
#include "Filter.h"
#include "CdShape.h"
#include "SphereTreeUtil.h"
#include "SphereTree.h"
#include "problem.h"
#include <Math/Physics.h>
#include <Math/Interpolator/ClampedCubicSplineArray.h>
#include <Math/Interpolator/PointToPointArray.h>
#include <Solver/SphereSpherePair.h>

using namespace motion_generator;
using namespace motion_interpolator;
using namespace hrp;

#define DI 0.02
void createDistanceConstraints(SphereTree &i_stree,
                               const std::vector<DistanceGeometry *>& i_shapes,
                               std::vector<DistanceConstraint *>& o_consts)
{
    // delete old constraints
    for (unsigned int i=0; i<o_consts.size(); i++){
        // TODO :
        delete o_consts[i];
    }
    o_consts.clear();

    int nconsts=0;
    double ds = 0.00;
    for (unsigned int i=0; i<i_shapes.size(); i++){
        std::vector<hrp::Vector3> spheres;
        Sphere *s = dynamic_cast<Sphere *>(i_shapes[i]);
        CappedCylinder *cp = dynamic_cast<CappedCylinder *>(i_shapes[i]);
        if (s){
            i_stree.collectSpheres(s->center(), s->radius(), DI,
                                   spheres);
#if 1
            for (unsigned int j=0; j<spheres.size(); j++){ 
                Sphere *o = new Sphere(i_stree.link(), spheres[j], i_stree.radius());
                DistancePair *dp = new SphereSpherePair(s, o);
                DistanceConstraint *dc = new DistanceConstraint(dp);
                dc->securityDistance(ds);
                dc->influenceDistance(DI);
                dc->xi(0.5*0.005);
                o_consts.push_back(dc);
            }
#endif
        }else if (cp){
            i_stree.collectSpheres(cp->point(), Vector3(cp->point()+cp->vector()),
                                   cp->radius(), DI, spheres);
#if 1
            for (unsigned int j=0; j<spheres.size(); j++){ 
                Sphere *o = new Sphere(i_stree.link(), spheres[j], i_stree.radius());
                DistancePair *dp = new CappedCylinderSpherePair(cp, o);
                DistanceConstraint *dc = new DistanceConstraint(dp);
                dc->securityDistance(ds);
                dc->influenceDistance(DI);
                dc->xi(0.5*0.005);
                o_consts.push_back(dc);
            }
#endif
        }
        nconsts += spheres.size();
    }
}

void loadPath(const char *filename, int dof, int &arm,
              std::vector<hrp::dvector> &qPath,
              std::vector<hrp::Vector3> &pPath, 
              std::vector<hrp::Vector3> &rpyPath)
{
    std::ifstream ifs("path.txt");
    hrp::dvector q(dof);
    hrp::Vector3 p, rpy;
    hrp::Matrix33 R;
    ifs >> arm >> p[0];
    while (!ifs.eof()){
        ifs >> p[1] >> p[2];
        for (int i=0; i<3; i++){
            for (int j=0; j<3; j++){
                ifs >> R(i,j);
            }
        }
        rpy = rpyFromRot(R);
        for (int i=0; i<dof; i++){
            ifs >> q[i];
        }
        qPath.push_back(q);
        pPath.push_back(p);
        rpyPath.push_back(rpy);
        ifs >> p[0];
    }
}

int main(int argc, char *argv[])
{
    const char *robotURL = NULL;
    bool avoidCollision=true, keepBalance=true, display=true;
    char *cloudf=NULL;
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
    loadPath("path.txt", robot->numJoints(), arm, qPath, pPath, rpyPath);

    Filter *filter = new Filter(robot, arm, keepBalance);
    filter->init(qPath[0], pPath[0], rotFromRpy(rpyPath[0]));

    std::vector<DistanceGeometry *> shapes;
    {
        std::ifstream ifs("hrp2.shape");
        std::string linkname;
        int stype;
        Vector3 p,p2,v;
        double r;
        ifs >> linkname;
        while (!ifs.eof()){
            ifs >> stype;
            Link *l = robot->link(linkname);
            if (!l && linkname[0] != '#'){
                std::cerr << "can't find a link named " << linkname << std::endl;
            }
            if (stype == CdShape::SPHERE){
                ifs >> p[0] >> p[1] >> p[2] >> r;
                if (l) {
                    Sphere *s = new Sphere(l, p, r);
                    shapes.push_back(s);
                }
            }else if (stype == CdShape::CAPSULE){
                ifs >> p[0] >> p[1] >> p[2] >> p2[0] >> p2[1] >> p2[2] >> r;
                v = p2 - p;
                if (l) {
                    CappedCylinder *s = new CappedCylinder(l, p, v, r);
                    shapes.push_back(s);
                }
            }
            ifs >> linkname;
        }
    }
    filter->setRobotShapes(shapes);

#if 0
    {
        std::ifstream ifs("hrp2.pairs");
        std::string name1, name2;
        ifs >> name1;
        while (!ifs.eof()){
            ifs >> name2;
            if (name1[0] == '#') continue;
            for (unsigned int i=0; i<shapes.size(); i++){
                DistanceGeometry *g1 = shapes[i];
                if (g1->link()->name == name1){
                    for (unsigned int j=0; j<shapes.size(); j++){
                        DistanceGeometry *g2 = shapes[j];
                        if (g2->link()->name == name2){
                            Sphere *s1 = dynamic_cast<Sphere *>(g1);
                            Sphere *s2 = dynamic_cast<Sphere *>(g2);
                            CappedCylinder *c1 = dynamic_cast<CappedCylinder *>(g1);
                            CappedCylinder *c2 = dynamic_cast<CappedCylinder *>(g2);
                            DistancePair *dp=NULL;
                            if (s1 && c2){
                                dp = new CappedCylinderSpherePair(c2, s1);
                            }else if (s2 && c1){
                                dp = new CappedCylinderSpherePair(c1, s2);
                            }else{
                                std::cerr << "distance pair between sphere and capsule is not supported"
                                          << std::endl;
                            }
                            if (dp){
                                DistanceConstraint *dc = new DistanceConstraint(dp);
                                dc->securityDistance(0);
                                dc->influenceDistance(0.1);
                                filter->addSelfCollisionAvoidanceConstraint(dc);
                            }
                        }
                    }
                }
            }
            ifs >> name1;
        }
    }
#endif

#if 1
    ClampedCubicSplineArray<int, hrp::dvector> *qTrj
        = new ClampedCubicSplineArray<int, hrp::dvector>(robot->numJoints());
    ClampedCubicSplineArray<int, hrp::Vector3> *pTrj
        = new ClampedCubicSplineArray<int, hrp::Vector3>(3);
    ClampedCubicSplineArray<int, hrp::Vector3> *rpyTrj
        = new ClampedCubicSplineArray<int, hrp::Vector3>(3);
#else
    PointToPointArray<int, hrp::dvector> *qTrj
        = new PointToPointArray<int, hrp::dvector>(robot->numJoints());
    PointToPointArray<int, hrp::Vector3> *pTrj
        = new PointToPointArray<int, hrp::Vector3>(3);
    PointToPointArray<int, hrp::Vector3> *rpyTrj
        = new PointToPointArray<int, hrp::Vector3>(3);
#endif

    // setup interpolator
    hrp::dvector q_old = qPath[0];
    qTrj->push_back(0, q_old);
    pTrj->push_back(0, pPath[0]);
    rpyTrj->push_back(0, rpyPath[0]);

    int totalFrames = 0;
    double maxOmega = 0.3;
    for (unsigned int i=1; i<qPath.size(); i++){
        const hrp::dvector &q = qPath[i];
        //
        double max=0;
        for (unsigned int j=0; j<q.size(); j++){
            double diff = fabs(q_old[j] - q[j]);
            if (diff > max) max = diff;
        }
#if 0
        totalFrames += max / maxOmega/DT + 1; // >= 1
        qTrj->push_back(totalFrames, q);
        pTrj->push_back(totalFrames, pPath[i]);
        rpyTrj->push_back(totalFrames, rpyPath[i]);
#else
        int nseg = max / maxOmega + 1;
        //std::cout << "nseg = " << nseg << std::endl;
        for (int j=1; j<=nseg; j++){
            //std::cout << "segt:" << ((max/maxOmega/DT)/nseg + 1)*DT << std::endl;
            totalFrames += (max/maxOmega/DT)/nseg + 1;
            qTrj->push_back(totalFrames, ((nseg-j)*q_old + j*q)/nseg);
            pTrj->push_back(totalFrames, Vector3(((nseg-j)*pPath[i-1] + j * pPath[i])/nseg));
            rpyTrj->push_back(totalFrames, Vector3(((nseg-j)*rpyPath[i-1] + j * rpyPath[i])/nseg));
        }
#endif
        q_old = q;
    }
    qTrj->update();
    pTrj->update();
    rpyTrj->update();
    std::cout << "duration of the motion:" << totalFrames*DT << std::endl;
    int frames = totalFrames;

    // viewer
    problem prob(0);
    std::vector<BodyPtr> obstacles;
    if (display){
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

#if 0
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
    while (frames > 0){
        printf("%6.3f/%6.3f\r", tm, totalFrames*DT);
        // compute reference motion
        qTrj->get(totalFrames-frames, q);
        pTrj->get(totalFrames-frames, p);
        rpyTrj->get(totalFrames-frames, rpy);
        frames--;

        disttm.begin();
        createDistanceConstraints(stree, shapes, consts);
        disttm.end();

        if (avoidCollision) filter->setDistanceConstraints(consts);

        // compute feasible motion
        qptm.begin();
        filter->filter(q, p, rotFromRpy(rpy));
        qptm.end();
        
        hrp::Vector3 com = robot->calcCM();
        comf << tm << " " << com[0] << " " << com[1] << " " << com[2] << std::endl;
        qf << tm;
        for (int i=0; i<robot->numJoints(); i++){
            qf << " " << robot->joint(i)->q;
        }
        qf << std::endl;

        tmf << tm << " " << disttm.time() << " " << qptm.time() << std::endl; 

        double minD = DI;
        for (unsigned int i=0; i<consts.size(); i++){
            hrp::Vector3 s, e;
            consts[i]->computeDistance();
            double d = consts[i]->distance();
            if (d < minD) minD = d;
        }
        if (minD < DI){
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
    }
    std::cout << "creating distance constraints(avg/max):"
              << disttm.averageTime()*1000 << "/" 
              << disttm.maxTime()*1000<< "[ms]" << std::endl;
    std::cout << "time for solving QP(avg/max):"
              << qptm.averageTime()*1000 << ","
              << qptm.maxTime()*1000 << "[ms]" << std::endl;

    return 0;
}

