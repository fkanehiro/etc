#include <octomap/octomap.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/Body.h>
#include <hrpModel/ColdetLinkPair.h>

using namespace std;
using namespace octomap;
using namespace hrp;

int main(int argc, char** argv) {
    if (argc < 2){
        std::cerr << "Usage: " << argv[0] << " [VRML file]" << std::endl;
        return 1;
    }

    BodyPtr cube = new Body();
    //const char *url = "/Users/fkanehiro/Dropbox/src/genogmap/cube.wrl";
    const char *url = "/home/kanehiro/Dropbox/src/genogmap/cube.wrl";
    if (!loadBodyFromModelLoader(cube, url, argc, argv, true)){
        std::cerr << "failed to load \"cube.wrl\"" << std::endl;
        return 1;
    }
    BodyPtr env = new Body();
    if (!loadBodyFromModelLoader(env, argv[1], argc, argv, true)){
        std::cerr << "failed to load(" << argv[1] << ")" << std::endl;
        return 1;
    }

    ColdetLinkPairPtr pair = new ColdetLinkPair(env->rootLink(), cube->rootLink());

    double res = 0.05;
    OcTree tree (res*2);  // create empty tree with resolution 0.1

    int nx = 6/res;
    int ny = 6/res;
    int nz = 2/res;

    // insert some measurements of occupied cells

    Link *l = cube->rootLink();
    for (int x=-nx; x<nx; x++) {
        for (int y=-ny; y<ny; y++) {
            for (int z=0; z<nz; z++) {
                l->p = Vector3(x*res, y*res, z*res);
                pair->updatePositions();
                point3d endpoint (l->p[0], l->p[1], l->p[2]);
                if (pair->detectIntersection()){
                    tree.updateNode(endpoint, true);
                }else{
                    tree.updateNode(endpoint, false);
                }
            }
        }
    }
    for (int i=-20; i<=20; i++){
        double y = i*res/4;
        point3d p(-2.2, y, 0.5);
        OcTreeNode* result = tree.search (p);
        if (!result){
            std::cout << y << " -1" << std::endl;
        }else{
            std::cout << y << " " << result->getOccupancy() << std::endl;
        }
    } 

    tree.writeBinary("test.bt");
}
