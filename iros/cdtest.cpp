#include "CdShapeUtil.h"
#include "SphereTree.h"
#include "SphereTreeUtil.h"
#include "problem.h"
#include "DistUtil.h"
#include <fstream>
#include <Model/HumanoidBodyUtil.h>
#include <hrpModel/Link.h>

using namespace motion_generator;

int main(int argc, char *argv[])
{
    // load robot
    const char *robotURL = "file:///home/kanehiro/openhrp3.0/Controller/IOserver/robot/HRP2/model/HRP2main.wrl";
    HumanoidBodyPtr robot = new HumanoidBody();
    loadHumanoidBodyFromModelLoader(robot, robotURL, argc, argv, false);

    // create robot shapes
    std::vector<CdShape> shapes;
    loadCdShapes(robot, "hrp2.shape", shapes);

    // create self-collision check pairs
    std::vector<std::pair<const CdShape *, const CdShape *> > pairs;
    loadCdPairs(shapes, "hrp2.pairs", pairs);

    problem prob;
    prob.addRobot("robot", robotURL, robot);
    hrp::BodyPtr obstacle = prob.addObstacle("obstacle", "file:///home/kanehiro/openhrp3.1/sample/model/house/table.main.wrl");
    obstacle->rootLink()->p = hrp::Vector3(-1,1,0.5);

    // create obstacle shape
    std::vector<hrp::Vector3> points;
    loadPointArray("table.pc", points);
    SphereTree T(obstacle->rootLink(), points, 0.01);
    T.updatePosition();

    prob.initOLV(argc, argv);
    prob.updateOLV();

    while (1){
        for (int i=12; i<robot->numJoints(); i++){
            hrp::Link *j = robot->joint(i);
            if (!j) continue;
            j->q = ((double)random())/RAND_MAX*(j->ulimit-j->llimit) + j->llimit;
        }
        robot->calcForwardKinematics();

        // update position of centers
        for (unsigned int i=0; i<shapes.size(); i++){
            shapes[i].updatePosition();
        }

        // robot <-> robot
        for (unsigned int i=0; i<pairs.size(); i++){
            const CdShape *s1 = pairs[i].first;
            const CdShape *s2 = pairs[i].second;
            if (s1->isColliding(s2)){
                std::cout << s1->link()->name << " and " << s2->link()->name
                          << " are colliding" << std::endl;
            }
        }
        // robot <-> obstacle
        for (unsigned int i=0; i<shapes.size(); i++){
            if (shapes[i].type() == CdShape::SPHERE){
                if (T.isColliding(shapes[i].center(), 
                                  shapes[i].radius())){
                    std::cout << shapes[i].link()->name << " is colliding" << std::endl;
                }
            }else{
                if (T.isColliding(shapes[i].center(0),
                                  shapes[i].center(1),
                                  shapes[i].radius())){
                    std::cout << shapes[i].link()->name << " is colliding" << std::endl;
                }
            }
        }
        prob.updateOLV();
        getchar();
    }
    
    return 0;
}
