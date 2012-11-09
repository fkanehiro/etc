#include <fstream>
#include <iostream>
#include <Solver/DistanceConstraint.h>
#include <Solver/Sphere.h>
#include <Solver/SphereSpherePair.h>
#include <Solver/CappedCylinder.h>
#include <Solver/CappedCylinderSpherePair.h>
#include <Solver/CappedCylinderCappedCylinderPair.h>
#include <Solver/SphereSpherePair.h>
#include "CdShape.h"
#include "SphereTree.h"
#include "Filter.h"
#include "ExecUtil.h"

using namespace hrp;
using namespace motion_generator;

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
    double ds = 0.005; // AR
    //double ds = 0.0;
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
    std::ifstream ifs(filename);
    std::cout << "loading " << filename << "..." << std::flush;
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
    std::cout << "done." << std::endl;
}

void loadShape(const char *filename, HumanoidBodyPtr robot,
               std::vector<DistanceGeometry *>& shapes)
{
    std::ifstream ifs(filename);
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

void setupSelfCollisionCheckPairs(const char *filename, 
                                  const std::vector<DistanceGeometry *>& shapes,
                                  Filter *filter)
{
    std::ifstream ifs(filename);
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
                        }else if (c1 && c2){
                            dp = new CappedCylinderCappedCylinderPair(c1, c2);
                        }else if (s1 && s2){
                            dp = new SphereSpherePair(s1, s2);
                        }else{
                            std::cerr << "unknown shape is detected"
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

int setupInterpolator(const std::vector<dvector>& qPath, 
                       const std::vector<Vector3>& pPath,
                       const std::vector<Vector3>& rpyPath,
                       dvInterpolator *qTrj,
                       v3Interpolator *pTrj,
                       v3Interpolator *rpyTrj)
{
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
    return totalFrames;
}
 
