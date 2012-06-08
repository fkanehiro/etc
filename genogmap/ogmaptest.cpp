#include <iostream> 
#include <octomap/octomap.h>

using namespace std;
using namespace octomap;

int main(int argc, char *argv[])
{
    double res = 0.1;
    OcTree tree (res);  // create empty tree with resolution 0.1
    
    point3d p1(-res/2,0,0);
    point3d p2(0,0,0);
    point3d p3(res/2,0,0);
    tree.updateNode(p1, false);
    tree.updateNode(p2, true);
    tree.updateNode(p3, false);
    
#define NDIV 10
    for (int i=-NDIV*2; i<=NDIV*2; i++){
        point3d p(res/NDIV*i, 0, 0);
        OcTreeNode *result = tree.search(p);
        std::cout << res/NDIV*i << " ";
        if (result){
            std::cout << result->getOccupancy() << std::endl;
        }else{
            std::cout << "?" << std::endl;
        }
    }

    return 0;
}
