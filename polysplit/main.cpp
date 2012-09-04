#include <iostream>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpUtil/OnlineViewerUtil.h>
#include <Util/OnlineViewerUtil.h>
#include "polysplit.h"

using namespace OpenHRP;
using namespace hrp;

const char url[] = "/home/kanehiro/openrtp/share/OpenHRP-3.1/robot/HRP2/model/HRP2main.wrl";
std::vector<std::string> linknames;
std::vector<anchor> anchors;

int main(int argc, char *argv[])
{
    OnlineViewer_var olv;
    olv = hrp::getOnlineViewer(argc, argv);
    olv->load("robot", url);

    BodyPtr body = BodyPtr(new Body());
    loadBodyFromModelLoader(body, url, argc, argv, true);
    body->setName("robot");
    convertToConvexHull(body);

    WorldState wstate;
    wstate.time = 0.0;
    wstate.characterPositions.length(1);
    setupCharacterPosition(wstate.characterPositions[0], body);

    linknames.push_back("CHEST_JOINT1");
    linknames.push_back("WAIST");
    linknames.push_back("RLEG_JOINT2");

    std::vector<Link *> links;
    for (size_t i=0; i<linknames.size(); i++){
        Link *l = body->link(linknames[i]);
        if (l){
            links.push_back(l);
        }else{
            std::cerr << "can't find a link named " << linknames[i] << std::endl;
        }
    }

    anchors.push_back(anchor(body->link("CHEST_JOINT1"), Vector3(0,0.2,0.5)));
    anchors.push_back(anchor(body->link("RLEG_JOINT2"), Vector3(-0.2,0,-0.2)));

    ColdetModelPtr chull = convertToConvexHull(links, anchors);
    std::vector<int> anchorVertices = findAnchors(chull, anchors);

    for (int i=0; i<10; i++){
        updateCharacterPosition(wstate.characterPositions[0], body);
        olv->update(wstate);
        wstate.time += 0.005;
    }
    
    return 0;
}
