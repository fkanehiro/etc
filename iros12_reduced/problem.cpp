#include <hrpModel/OnlineViewerUtil.h>
#include <hrpUtil/OnlineViewerUtil.h>
#include <hrpModel/Link.h>
#include "problem.h"

problem::problem()
{
    m_planner.initPlanner("localhost:2809");
}


hrp::BodyPtr problem::addRobot(const std::string& i_name, 
                               const std::string &i_url,
                               hrp::BodyPtr i_body)
{
    m_robot = addBody(i_name, i_url, i_body);
    m_planner.setRobotName(i_name);
    return m_robot;
}

hrp::BodyPtr problem::addGoal(const std::string& i_name, 
                              const std::string &i_url,
                              hrp::BodyPtr i_body)
{
    m_goal = addBody(i_name, i_url, i_body);
    return m_goal;
}

hrp::BodyPtr problem::addObstacle(const std::string& i_name, 
                                  const std::string &i_url,
                                  hrp::BodyPtr i_body)
{
    hrp::BodyPtr body = addBody(i_name, i_url, i_body);
#if 1
    std::cout << "the number of triangles:" << body->rootLink()->coldetModel->getNumTriangles() << std::endl;
#endif
    m_obstacles.push_back(body);
    return body;
}

hrp::BodyPtr problem::addBody(const std::string& i_name, 
                              const std::string &i_url,
                              hrp::BodyPtr i_body)
{
    m_names.push_back(i_name);
    m_urls.push_back(i_url);
    if (i_body){
        m_planner.registerCharacter(i_name.c_str(), i_body);
        return i_body;
    }else{
        hrp::BodyPtr body = m_planner.registerCharacterByURL(i_name.c_str(), 
                                                           i_url.c_str());
        return body;
    }
}

void problem::initCollisionCheckPairs(bool checkSelfCollision)
{
    double tolerance = 0.0;
    for (unsigned int i=0; i<m_obstacles.size(); i++){
        hrp::BodyPtr obj = m_obstacles[i];
        for (int i=0; i<m_robot->numLinks(); i++){
            for (int j=0; j<obj->numLinks(); j++){
                m_planner.registerIntersectionCheckPair(
                    m_robot->name().c_str(),
                    m_robot->link(i)->name.c_str(),
                    obj->name().c_str(),
                    obj->link(j)->name.c_str(), 
                    tolerance);
            }
        }
    }

    if (!checkSelfCollision) return;

    for (int i=0; i<m_robot->numLinks(); i++){
        for (int j=i+1; j<m_robot->numLinks(); j++){
            hrp::Link *l1 = m_robot->link(i);
            hrp::Link *l2 = m_robot->link(j);
            if (l1->parent == l2 || l2->parent == l1) continue; 
            m_planner.registerIntersectionCheckPair(m_robot->name().c_str(),
                                                  l1->name.c_str(),
                                                  m_robot->name().c_str(),
                                                  l2->name.c_str(), tolerance);
        }
    }
}

void problem::initOLV(int argc, char *argv[])
{
    olv = hrp::getOnlineViewer(argc, argv);

    for (unsigned int i=0; i<m_urls.size(); i++){
        olv->load(m_names[i].c_str(), m_urls[i].c_str());
    }
    olv->clearLog();

    wstate.time = 0.0;

    m_bodies = m_obstacles;
    m_bodies.push_back(m_robot);
    if (m_goal) m_bodies.push_back(m_goal);

    wstate.characterPositions.length(m_bodies.size());
    for (unsigned int i=0; i<m_bodies.size(); i++){
        setupCharacterPosition(wstate.characterPositions[i], m_bodies[i]);
    }
}

void problem::updateOLV()
{
    for (unsigned int i=0; i<m_bodies.size(); i++){
        updateCharacterPosition(wstate.characterPositions[i], m_bodies[i]);
    }
    olv->update(wstate);
    wstate.time += 0.005;
}

#define LENGTH2DEPTH(x)	((x)-0.1)/10
void updateLineSegment(OpenHRP::CollisionPoint& cp, const hrp::Vector3& p1, const hrp::Vector3& p2)
{
    hrp::Vector3 dv(p2 - p1);
    double l = dv.norm();
    hrp::Vector3 n(dv/l);
    cp.position[0] = p1[0];
    cp.position[1] = p1[1];
    cp.position[2] = p1[2];
    cp.normal[0] = n[0];
    cp.normal[1] = n[1];
    cp.normal[2] = n[2];
    cp.idepth = LENGTH2DEPTH(l);
}

void problem::updateOLV(const std::vector<hrp::Vector3>& i_starts,
                        const std::vector<hrp::Vector3>& i_ends)
{
    wstate.collisions.length(1);
    wstate.collisions[0].points.length(i_starts.size());
    for (unsigned int i=0; i<i_starts.size(); i++){
        updateLineSegment(wstate.collisions[0].points[i], i_starts[i], i_ends[i]);
    }
    updateOLV();
}


void problem::initPlanner()
{
    m_planner.initSimulation();
}

bool problem::checkCollision()
{
    return m_planner.checkCollision();
}
