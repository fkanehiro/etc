#include <iostream>
#include <fstream>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/Link.h>
#include <hrpModel/JointPath.h>
#include <hrpModel/ColdetLinkPair.h>
#include <hrpUtil/OnlineViewerUtil.h>
#include <Util/OnlineViewerUtil.h>

using namespace hrp;
using namespace OpenHRP;

int main(int argc, char *argv[])
{
    if (argc < 3){
        std::cerr << "Usage: " << argv[0] << " vrml1 vrml2 " << std::endl;
        return 1;
    }

    BodyPtr body1(new Body());
    BodyPtr body2(new Body());
    if (!loadBodyFromModelLoader(body1, argv[1], argc, argv, true)){
        std::cerr << "failed to load " << argv[1] << std::endl;
        return 1;
    }
    if (!loadBodyFromModelLoader(body2, argv[2], argc, argv, true)){
        std::cerr << "failed to load " << argv[1] << std::endl;
        return 1;
    }

    body1->calcForwardKinematics();
    body2->calcForwardKinematics();

    OpenHRP::OnlineViewer_var olv;
    olv = getOnlineViewer(argc, argv);
    olv->load(body1->modelName().c_str(), argv[1]);
    olv->load(body2->modelName().c_str(), argv[2]);
    olv->clearLog();

    OpenHRP::WorldState wstate;
    wstate.time = 0.0;
    wstate.characterPositions.length(2);
    setupCharacterPosition(wstate.characterPositions[0], body1);
    updateCharacterPosition(wstate.characterPositions[0], body1);
    setupCharacterPosition(wstate.characterPositions[1], body2);
    updateCharacterPosition(wstate.characterPositions[1], body2);
    wstate.collisions.length(1);
    
    ColdetLinkPairPtr pair = new ColdetLinkPair(body1->rootLink(),
                                                body2->rootLink());
    pair->enableNormalVectorCorrection(false);
    pair->updatePositions();
    std::vector<collision_data> cd = pair->detectCollisions();
    
    std::cout << "no. of triangle pairs=" << cd.size() << std::endl;

    CollisionPointSequence &cps = wstate.collisions[0].points;
    int idx=0;
    for (unsigned int i=0; i<cd.size(); i++){
      collision_data& data = cd[i];
      std::cout << "no. of intersecting points=" << data.num_of_i_points << std::endl;
      std::cout << "n:" << data.n.transpose() << std::endl;
      cps.length(cps.length() + data.num_of_i_points);
      for (int j=0; j<data.num_of_i_points; j++){
        CollisionPoint &cp = cps[idx++];
        cp.idepth = data.depth;
        std::cout << "p" << j << ":" << data.i_points[j].transpose() << std::endl;
        cp.position[0] = data.i_points[j][0];
        cp.position[1] = data.i_points[j][1];
        cp.position[2] = data.i_points[j][2];
        cp.normal[0] = data.n[0];
        cp.normal[1] = data.n[1];
        cp.normal[2] = data.n[2];
        cp.idepth = 0.1;
      }
    }

    olv->update(wstate);
    
    return 0;
}
