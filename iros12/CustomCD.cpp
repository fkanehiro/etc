#include <iostream>
#include <hrpUtil/TimeMeasure.h>
#include "CustomCD.h"
#include "CdShapeUtil.h"
#include "SphereTreeUtil.h"

CustomCD::CustomCD(hrp::BodyPtr i_robot, 
                   const char *i_shapefile, const char *i_pairfile,
                   hrp::BodyPtr i_env,
                   const char *i_envpcfile) : m_stree(NULL), m_outerTree(false)
{
    loadCdShapes(i_robot, i_shapefile, m_shapes);
    loadCdPairs(m_shapes, i_pairfile, m_pairs);
    loadPointCloud(i_env, i_envpcfile);
}

CustomCD::CustomCD(hrp::BodyPtr i_robot, 
                   const char *i_shapefile, const char *i_pairfile,
                   hrp::BodyPtr i_env,
                   SphereTree *i_stree) : m_stree(i_stree), m_outerTree(true)
{
    loadCdShapes(i_robot, i_shapefile, m_shapes);
    loadCdPairs(m_shapes, i_pairfile, m_pairs);
}

CustomCD::~CustomCD()
{
    if (!m_outerTree) delete m_stree;
}

void CustomCD::updatePositions()
{
    m_stree->updatePosition();
    for (unsigned int i=0; i<m_shapes.size(); i++){
        m_shapes[i].updatePosition();
    }
}

bool CustomCD::checkCollision()
{
    return ::checkCollision(*m_stree, m_shapes, m_pairs);
}

bool CustomCD::loadPointCloud(hrp::BodyPtr i_env, const char *i_envpcfile)
{
    if (m_stree) delete m_stree;

    std::vector<hrp::Vector3> points;
    loadPointArray(i_envpcfile, points);
    TimeMeasure tm;
    tm.begin();
    m_stree = new SphereTree(i_env->rootLink(), points, 0.01);
    tm.end();
    std::cout << "time to construct sphere tree:" << tm.time()*1000 << "[ms]"
              << std::endl;
    
}

