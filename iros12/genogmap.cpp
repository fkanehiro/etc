#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/Body.h>
#include <hrpModel/ColdetLinkPair.h>
#include <octomap/octomap.h>

using namespace std;
using namespace octomap;
using namespace hrp;

int main(int argc, char** argv) {
    if (argc < 2){
        std::cerr << "Usage: " << argv[0] << " [VRML file] [-vrml]" << std::endl;
        return 1;
    }
    bool vrml = false;
    for (int i=2; i<argc; i++){
        if (strcmp(argv[i],"-vrml")==0){
            vrml = true;
        }
    }

    BodyPtr cube = BodyPtr(new Body());
    const char *url = "/home/kanehiro/Dropbox/src/iros/cube.wrl";
    if (!loadBodyFromModelLoader(cube, url, argc, argv, true)){
        std::cerr << "failed to load \"cube.wrl\"" << std::endl;
        return 1;
    }
    BodyPtr env = BodyPtr(new Body());
    if (!loadBodyFromModelLoader(env, argv[1], argc, argv, true)){
        std::cerr << "failed to load(" << argv[1] << ")" << std::endl;
        return 1;
    }
    env->rootLink()->p.setZero();

    ColdetLinkPairPtr pair = new ColdetLinkPair(env->rootLink(), cube->rootLink());


#if 0 // plant
    double res = 0.02; // size of cube
    double startx = -0.5, endx = 1.0;
    double starty = -1.0, endy = 1.0;
    double startz =  0.0, endz = 1.6;
#endif
#if 0 // table
    double res = 0.02; // size of cube
    double startx = -0.6, endx = 0.6;
    double starty = -1.1, endy = 1.1;
    double startz = 0, endz = 1;
#endif
#if 1 // 3301
    double res = 0.05; // size of cube
    double startx = -6, endx = 1.5;
    double starty = -1.5, endy = 4;
    double startz = 0, endz = 1.0;
#endif
    double dl = res/2;
    int nx = (endx - startx)/dl+1;
    int ny = (endy - starty)/dl+1;
    int nz = (endz - startz)/dl+1;

    OcTree tree (res);

    Link *l = cube->rootLink();
    int n = 0;
    for (int i=0; i<nx; i++) {
        for (int j=0; j<ny; j++) {
            for (int k=0; k<nz; k++) {
                l->p = Vector3(startx+i*dl, starty+j*dl, startz+k*dl);
                point3d endpoint (l->p[0], l->p[1], l->p[2]);
                pair->updatePositions();
                if (pair->detectIntersection()){
                    n++;
                    if (vrml){
                        std::cout << "Transform {" << std::endl;
                        std::cout << "  translation " << l->p[0] << " "
                                  << l->p[1] << " " << l->p[2] << std::endl;
                        std::cout << "  children Shape {" << std::endl;
                        std::cout << "    geometry Box {" << std::endl;
                        std::cout << "        size " << res << " " << res << " " << res << std::endl;
                        std::cout << "    }" << std::endl;
                        std::cout << "    appearance Appearance {" << std::endl;
                        std::cout << "        material Material {" << std::endl;
                        std::cout << "            diffuseColor 1.0 1.0 0.0" << std::endl;
                        std::cout << "        }" << std::endl;
                        std::cout << "    }" << std::endl;
                        std::cout << "  }" << std::endl;
                        std::cout << "}" << std::endl;
                    }else{
                        std::cout << l->p[0] << " " << l->p[1] << " " 
                                  << l->p[2] << std::endl;
                    }
                    tree.updateNode(endpoint, true);
                }else{
                    tree.updateNode(endpoint, false);
                }
            }
        }
    }
    std::cerr << n << "voxels are created" << std::endl;
    tree.writeBinary("test.bt");
}
