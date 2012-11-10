#include <iostream>
#include <hrpModel/Link.h>
#include "DistUtil.h"
#include "SphereTree.h"

void computeBoundingBox(const std::vector<const hrp::Vector3 *> &i_points,
                        hrp::Vector3 &o_min, hrp::Vector3 &o_max,
                        hrp::Vector3 &o_avg) 
{
    if (!i_points.size()){
        std::cerr << "points.size() = 0!!" << std::endl;
    }
    o_min = o_max = o_avg = *i_points[0];
    for (unsigned int i=1; i<i_points.size(); i++){
        const hrp::Vector3 &p = *i_points[i];
        for (int j=0; j<3; j++){
            if (p[j] > o_max[j]) o_max[j] = p[j];
            if (p[j] < o_min[j]) o_min[j] = p[j];
        }
        o_avg += p;
    }
    o_avg /= i_points.size();
}

void split(const std::vector<const hrp::Vector3 *> &i_points, int i_axis, double i_p,
           std::vector<const hrp::Vector3 *> &o_points1, 
           std::vector<const hrp::Vector3 *> &o_points2)
{
    for (unsigned int i=0; i<i_points.size(); i++){
        const hrp::Vector3 *ptr = i_points[i];
        if ((*ptr)[i_axis] > i_p){
            o_points1.push_back(ptr);
        }else{
            o_points2.push_back(ptr);
        }
    }
}

SphereTreeNode::SphereTreeNode(const std::vector<const hrp::Vector3 *> &i_points, double i_radius)
{
    //std::cout << "SphereTreeNode::SphereTreeNode(" << i_points.size() << ")" << std::endl;
    if (i_points.size() == 0){
        std::cerr << "i_points.size() == 0" << std::endl;
    }
    m_isLeaf = i_points.size() == 1;
    if (m_isLeaf){
        m_radius = i_radius;
        m_center = *i_points[0];
        m_child[0] = m_child[1] = NULL;
    }else{
        // determine split axis and position
        hrp::Vector3 max, min, avg;
        computeBoundingBox(i_points, min, max, avg);
        for (int i=0; i<3; i++){
            max[i] += i_radius;
            min[i] -= i_radius;
        }
        hrp::Vector3 len(max - min);
        //std::cout << "bbox size:" << len << std::endl;
        double r1 = len.norm()/2;
        //std::cout << "len = " << len << std::endl;
        int axis = 0;
        if (len[1] > len[axis]) axis = 1; 
        if (len[2] > len[axis]) axis = 2; 
        //std::cout << "axis = " << axis << ", pos = " << avg[axis] << std::endl;

        // create children
        std::vector<const hrp::Vector3 *> points[2];
        split(i_points, axis, avg[axis], points[0], points[1]);
        //std::cout << i_points.size() << "->" << points[0].size() << "+" << points[1].size() << std::endl;
        for (int i=0; i<2; i++){
            m_child[i] = new SphereTreeNode(points[i], i_radius);
        }

        // compute radius and center
        hrp::Vector3 dv(m_child[1]->center() - m_child[0]->center());
        double l = dv.norm();
        double r2 = (m_child[0]->radius() + m_child[1]->radius() + l)/2;
        if (r1 > r2){
            m_radius = r2;
            m_center = (m_child[0]->center() + m_child[1]->center()
                        + (m_child[1]->radius() - m_child[0]->radius())/l*dv)/2;
        }else{
            m_radius = r1;
            m_center = (max + min)/2;
        }
    }
}

SphereTreeNode::~SphereTreeNode()
{
    for (int i=0; i<2; i++){
        if (m_child[i]) delete m_child[i];
    }
}

SphereTree::SphereTree(hrp::Link *i_link, 
                       const std::vector<hrp::Vector3> &i_points,
                       double i_radius)
    : m_link(i_link), m_radius(i_radius), m_root(NULL)
{
    build(i_points, i_radius);
}

SphereTree::~SphereTree()
{
    delete m_root;
}

void SphereTree::build(const std::vector<hrp::Vector3> &i_points,
                       double i_radius)
{
    if (m_root) delete m_root;

    std::vector<const hrp::Vector3 *> points;
    for (unsigned int i=0; i<i_points.size(); i++){
        const hrp::Vector3 &v = i_points[i];
        points.push_back(&v);
    }
    m_p.setZero();
    m_R <<
        1,0,0,
        0,1,0,
        0,0,1;
    if (points.size()){
        m_root = new SphereTreeNode(points, m_radius);
    }else{
        m_root = NULL;
    }
}

bool SphereTree::isColliding(const hrp::Vector3 &i_p, double i_r,
                             double i_tolerance) const
{
    hrp::Vector3 p(m_R.transpose()*(i_p - m_p));
    return m_root->isColliding(p, i_r, i_tolerance);
}

double SphereTree::distance(const hrp::Vector3 &i_p, double i_r) const
{
    hrp::Vector3 p(m_R.transpose()*(i_p - m_p));
    double minD = std::numeric_limits<double>::max();
    m_root->distance(p, i_r, minD);
    return minD;
}

bool SphereTreeNode::isColliding(const hrp::Vector3 &i_p, double i_r,
                                 double i_tolerance)
{
    double d = (i_p - m_center).norm();
    if (d < i_r + m_radius + i_tolerance){
        if (m_isLeaf){
            return true;
        }else{
            if (m_child[0]->isColliding(i_p, i_r)) return true;
            if (m_child[1]->isColliding(i_p, i_r)) return true;
        }
    }
    return false;
}

void SphereTreeNode::distance(const hrp::Vector3 &i_p, double i_r, 
                              double &io_minD)
{
    double d = (i_p - m_center).norm() - (i_r + m_radius);
    if (io_minD > d){
        if (m_isLeaf){
            io_minD = d;
        }else{
            m_child[0]->distance(i_p, i_r, io_minD);
            m_child[1]->distance(i_p, i_r, io_minD);
        }
    }
}

void SphereTree::collectSpheres(const hrp::Vector3& i_p, 
                                double i_r, double i_d,
                                std::vector<hrp::Vector3> &o_centers)
{
    if (!m_root) return;

    hrp::Vector3 p(m_R.transpose()*(i_p - m_p));
    std::vector<SphereTreeNode *> spheres;
    m_root->collectSpheres(p, i_r, i_d, spheres);
    for (unsigned int i=0; i<spheres.size(); i++){
        o_centers.push_back(spheres[i]->center());
    }
}

void SphereTreeNode::collectSpheres(const hrp::Vector3 &i_p, 
                                    double i_r, double i_d,
                                    std::vector<SphereTreeNode *> &o_spheres)
{
    double d = (i_p - m_center).norm();
    if (d < i_r + m_radius + i_d){
        if (m_isLeaf){
            o_spheres.push_back(this);
        }else{
            for (int i=0; i<2; i++){
                m_child[i]->collectSpheres(i_p, i_r, i_d, o_spheres);
            }
        }
    }
}

bool SphereTree::isColliding(const hrp::Vector3 &i_p1,
                             const hrp::Vector3 &i_p2,
                             double i_r, double i_tolerance) const
{
    hrp::Vector3 p1(m_R.transpose()*(i_p1 - m_p));
    hrp::Vector3 p2(m_R.transpose()*(i_p2 - m_p));
    return m_root->isColliding(p1, p2, i_r, i_tolerance);
}

double SphereTree::distance(const hrp::Vector3 &i_p1,
                            const hrp::Vector3 &i_p2,
                            double i_r) const
{
    hrp::Vector3 p1(m_R.transpose()*(i_p1 - m_p));
    hrp::Vector3 p2(m_R.transpose()*(i_p2 - m_p));
    double minD = std::numeric_limits<double>::max();
    m_root->distance(p1, p2, i_r, minD);
    return minD;
}

bool SphereTreeNode::isColliding(const hrp::Vector3 &i_p1,
                                 const hrp::Vector3 &i_p2,
                                 double i_r, double i_tolerance)
{
    double d = pointLineSegmentDistance(m_center, i_p1, i_p2);
    if (d < i_r + m_radius + i_tolerance){
        if (m_isLeaf){
            return true;
        }else{
            if (m_child[0]->isColliding(i_p1, i_p2, i_r)) return true;
            if (m_child[1]->isColliding(i_p1, i_p2, i_r)) return true;
        }
    }
    return false;
}

void SphereTreeNode::distance(const hrp::Vector3 &i_p1,
                              const hrp::Vector3 &i_p2,
                              double i_r, double& io_minD)
{
    double d = pointLineSegmentDistance(m_center, i_p1, i_p2)
        - (i_r + m_radius);
    if (io_minD > d){
        if (m_isLeaf){
            io_minD = d;
        }else{
            m_child[0]->distance(i_p1, i_p2, i_r, io_minD);
            m_child[1]->distance(i_p1, i_p2, i_r, io_minD);
        }
    }
}

void SphereTree::collectSpheres(const hrp::Vector3& i_p1, 
                                const hrp::Vector3& i_p2, 
                                double i_r, double i_d,
                                std::vector<hrp::Vector3> &o_centers)
{
    if (!m_root) return;

    hrp::Vector3 p1(m_R.transpose()*(i_p1 - m_p));
    hrp::Vector3 p2(m_R.transpose()*(i_p2 - m_p));
    std::vector<SphereTreeNode *> spheres;
    m_root->collectSpheres(p1, p2, i_r, i_d, spheres);
    for (unsigned int i=0; i<spheres.size(); i++){
        o_centers.push_back(spheres[i]->center());
    }
}

void SphereTreeNode::collectSpheres(const hrp::Vector3 &i_p1, 
                                    const hrp::Vector3 &i_p2,
                                    double i_r, double i_d,
                                    std::vector<SphereTreeNode *> &o_spheres)
{
    double d = pointLineSegmentDistance(m_center, i_p1, i_p2);
    if (d < i_r + m_radius + i_d){
        if (m_isLeaf){
            o_spheres.push_back(this);
        }else{
            for (int i=0; i<2; i++){
                m_child[i]->collectSpheres(i_p1, i_p2, i_r, i_d, o_spheres);
            }
        }
    }
}

void SphereTree::updatePosition()
{
    m_p = m_link->p;
    m_R = m_link->R;
}

