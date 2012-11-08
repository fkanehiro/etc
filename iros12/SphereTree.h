#ifndef __STREE_H__
#define __STREE_H__

#include <vector>
#include <hrpUtil/Eigen3d.h>

namespace hrp{
    class Link;
};

class SphereTreeNode{
public:
    SphereTreeNode(const std::vector<const hrp::Vector3 *> &i_points,
                   double i_raidus);
    ~SphereTreeNode();
    bool isLeaf() const { return m_isLeaf; }
    double radius() { return m_radius; }
    const hrp::Vector3 &center() { return m_center; }
    SphereTreeNode *child(int i){ return m_child[i]; }
    bool isColliding(const hrp::Vector3& i_p, double i_r);
    void collectSpheres(const hrp::Vector3& i_p, double i_r, double i_d,
                        std::vector<SphereTreeNode *> &o_spheres);
    bool isColliding(const hrp::Vector3& i_p1, 
                     const hrp::Vector3& i_p2, double i_r);
    void collectSpheres(const hrp::Vector3& i_p1, const hrp::Vector3& i_p2,
                        double i_r, double i_d,
                        std::vector<SphereTreeNode *> &o_spheres);
private:
    bool m_isLeaf;
    SphereTreeNode *m_child[2];
    double m_radius;
    hrp::Vector3 m_center;
};

class SphereTree{
public:
    SphereTree(hrp::Link *i_link,
               const std::vector<hrp::Vector3> &i_points,
               double i_radius);
    ~SphereTree();
    SphereTreeNode *root() { return m_root; }
    /**
     * @brief collision detection against a sphere
     * @param i_p position of the sphere
     * @param i_r radius of the sphere
     * @return true if colliding
     */
    bool isColliding(const hrp::Vector3& i_p, double i_r) const;
    /**
     * @brief collect spheres in the distance bound of a sphere
     * @param i_p position of the sphere
     * @param i_r radius of the sphere
     * @param i_d thickness of the distance bound
     * @param o_centers vector of positions of spheres in the distance bound
     */
    void collectSpheres(const hrp::Vector3& i_p, double i_r, double i_d,
                        std::vector<hrp::Vector3> &o_centers);
    /**
     * @brief collision detection against a capsule
     * @param i_p1 an end point of the line segment in the capsule
     * @param i_p2 the other end point of the line segment in the capsule
     * @param i_r radius of the capsule
     * @return true if colliding
     */
    bool isColliding(const hrp::Vector3& i_p1, const hrp::Vector3& i_p2, 
                     double i_r) const;
    /**
     * @brief collect spheres in the distance bound of a capsule
     * @param i_p1 an end point of the line segment in the capsule
     * @param i_p2 the other end point of the line segment in the capsule
     * @param i_r radius of the capsule
     * @param i_d thickness of the distance bound
     * @param o_centers vector of positions of spheres in the distance bound
     */
    void collectSpheres(const hrp::Vector3& i_p1, const hrp::Vector3& i_p2,
                        double i_r, double i_d,
                        std::vector<hrp::Vector3> &o_centers);
    void updatePosition();
    double radius() const { return m_radius; }
    hrp::Link *link() { return m_link; }
private:
    SphereTreeNode *m_root;
    double m_radius;
    hrp::Link *m_link;
    hrp::Vector3 m_p;
    hrp::Matrix33 m_R;
};

#endif
