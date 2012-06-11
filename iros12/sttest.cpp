#include <iostream>
#include <fstream>
#include "SphereTree.h"

void indent(int level)
{
    for (int i=0; i<level*2; i++){
        std::cout << " ";
    }
}

void dump(SphereTreeNode *node, int level=0)
{
    indent(level);
    std::cout << "level = " << level 
              << ", radius = " << node->radius()
              << ", center = (" << node->center()[0] << " " 
              << node->center()[1] << " "
              << node->center()[2] << ")" << std::endl;
    if (node->isLeaf()) return;
    for (int i=0; i<2; i++){
        dump(node->child(i), level+1);
    }
}

void vrml(SphereTreeNode *node)
{
    double r = node->radius();
    std::cout << "Transform{" << std::endl;
    std::cout << "  translation " << node->center()[0] << " " << node->center()[1] << " " << node->center()[2] << std::endl;
    std::cout << "  children Shape{" << std::endl;
#if 0
    std::cout << "    geometry Sphere {" << std::endl;
    std::cout << "        radius " << r << std::endl;
    std::cout << "    }" << std::endl;
#else
    std::cout << "    geometry Box {" << std::endl;
    std::cout << "        size " << r*2 << " " << r*2 << " " << r*2 << std::endl;
    std::cout << "    }" << std::endl;
#endif
    std::cout << "    appearance Appearance {" << std::endl;
    std::cout << "        material Material {" << std::endl;
    std::cout << "            diffuseColor 0.0 0.0 1.0" << std::endl;
    std::cout << "        }" << std::endl;
    std::cout << "    }" << std::endl;
    std::cout << "  }" << std::endl;
    std::cout << "}" << std::endl;
}

void vrml(SphereTreeNode *node, int target, int level=0)
{
    if (level == target || (target==-1&&node->isLeaf())){
        vrml(node);
    }

    if (!node->isLeaf()){
        for (int i=0; i<2; i++){
            vrml(node->child(i), target, level+1);
        }
    }
}


int main(int argc, char *argv[])
{
    if (argc < 2){
        std::cerr << argv[0] << "[point cloud]" << std::endl;
        return 1;
    }
    std::vector<hrp::Vector3> points;

    std::ifstream ifs(argv[1]);

    std::cerr << "loading..." << std::flush;
    hrp::Vector3 p;
    ifs >> p[0];
    while (!ifs.eof()){
        ifs >> p[1] >> p[2];
        points.push_back(p);
        ifs >> p[0];
    }
    std::cerr << "done(" << points.size() << "points)." << std::endl;
    SphereTree T(NULL, points, 0.01);
    
    //dump(T.root());
    vrml(T.root(), -1);

#if 0
    hrp::Vector3 v(0.3, 0, 0.1);
    double r = 0.1;
    for (int i=0; i<20; i++){
        v[2] += 0.01;
        std::cout << "z=" << v[2] << ", isColliding=" << T.isColliding(v, r) << std::endl;
    }
#endif

#if 0    
    hrp::Vector3 v(0,0,0.1);
    double r = 0.1;
    std::vector<SphereTreeNode *> nodes;
    T.root()->collectSpheres(v, r, 0, nodes);
    for (unsigned int i=0; i<nodes.size(); i++){
        vrml(nodes[i]);
    }
#endif

#if 0
    hrp::Vector3 p1( 0.4, 0.1,0.1);
    hrp::Vector3 p2(-0.4,-0.1,0.1);
    double r = 0.02;
    std::vector<SphereTreeNode *> nodes;
    T.root()->collectSpheres(p1, p2, r, 0, nodes);
    for (unsigned int i=0; i<nodes.size(); i++){
        vrml(nodes[i]);
    }
#endif

    return 0;
}
