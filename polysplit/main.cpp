#include <iostream>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpUtil/OnlineViewerUtil.h>
#include <Util/OnlineViewerUtil.h>
#include <util/BVutil.h>
#include "polysplit.h"

using namespace OpenHRP;
using namespace hrp;

const char url[] = "/home/kanehiro/openrtp/share/OpenHRP-3.1/robot/HRP2/model/HRP2main.wrl";
std::vector<std::string> linknames;

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
    wstate.collisions.length(1);

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

    std::vector<Anchor> anchors;
    anchors.push_back(Anchor(body->link("CHEST_JOINT1"), Vector3(0,0.2,0.5)));
    anchors.push_back(Anchor(body->link("RLEG_JOINT2"), Vector3(-0.2,0,-0.2)));
    String st(&anchors[0], &anchors[1]);
        
    ColdetModelPtr chull = convertToConvexHull(links, st);
    if (!checkAnchors(chull, anchors)){
        std::cerr << "at least one of anchors is not on the surface of the convex hull" << std::endl;
        return 1;
    }

    Plane plane(st.first->position(), st.second->position(), 0);
    LineSegmentArray lsa = split(chull, plane);
    updateCharacterPosition(wstate.characterPositions[0], body);
    wstate.collisions[0].points.length(lsa.size());
    for (size_t i=0; i<lsa.size(); i++){
        updateLineSegment(wstate.collisions[0].points[i],
                          lsa[i].first.cast<double>(), 
                          lsa[i].second.cast<double>());
    }
    olv->update(wstate);
    wstate.time += 0.005;
    
    return 0;
}
