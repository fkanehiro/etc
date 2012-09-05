#include <iostream>
#include <fstream>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/Link.h>
#include <hrpUtil/OnlineViewerUtil.h>
#include <Util/OnlineViewerUtil.h>
#include <util/BVutil.h>
#include "polysplit.h"

using namespace OpenHRP;
using namespace hrp;

const char url[] = "/home/kanehiro/openrtp/share/OpenHRP-3.1/robot/HRP4Cg/model/HRP4Cg_main.wrl";
const char logfile[] = "1625_smartsuit_0kg_trial01-astate.log";

int main(int argc, char *argv[])
{
    OnlineViewer_var olv;
    olv = hrp::getOnlineViewer(argc, argv);
    olv->load("robot", url);
    olv->clearLog();

    BodyPtr body = BodyPtr(new Body());
    loadBodyFromModelLoader(body, url, argc, argv, true);
    body->setName("robot");
    convertToConvexHull(body);

    WorldState wstate;
    wstate.time = 0.0;
    wstate.characterPositions.length(1);
    setupCharacterPosition(wstate.characterPositions[0], body);
    wstate.collisions.length(3);

    // ワイヤの固定部
    std::vector<Anchor> anchors;
    anchors.push_back(Anchor(body->link("CHEST_Y"), Vector3(-0.03, 0.1,0.35)));
    anchors.push_back(Anchor(body->link("R_HIP_P"), Vector3(-0.08,0,-0.2)));
    anchors.push_back(Anchor(body->link("CHEST_Y"), Vector3(-0.03,-0.1,0.35)));
    anchors.push_back(Anchor(body->link("L_HIP_P"), Vector3(-0.08,0,-0.2)));

    wstate.collisions.length(anchors.size()+1);
    for (size_t i=1; i<wstate.collisions.length(); i++){
        setupFrame(wstate.collisions[i], 0.05);
    }

    // ワイヤと接触するリンクセットその1
    std::vector<Link *> linkset1;
    linkset1.push_back(body->link("CHEST_Y"));
    linkset1.push_back(body->link("WAIST"));
    linkset1.push_back(body->link("R_HIP_P"));

    // ワイヤと接触するリンクセットその2
    std::vector<Link *> linkset2;
    linkset2.push_back(body->link("CHEST_Y"));
    linkset2.push_back(body->link("WAIST"));
    linkset2.push_back(body->link("L_HIP_P"));

    // ワイヤ（両端点及びリンクセット）の定義
    std::vector<String> strings;
    strings.push_back(String(&anchors[0], &anchors[1], linkset1));
    strings.push_back(String(&anchors[2], &anchors[3], linkset2));


    std::ifstream ifs(logfile);
    if (!ifs.is_open()){
        std::cerr << "failed to open the log file:" << logfile << std::endl;
        return 1;
    }
    char buf[1024];
    ifs.getline(buf, 1024); // skip header
    double q;

    while(1){
        for (int i=0; i<body->numJoints(); i++){
            ifs >> q;
            if (ifs.eof()) return 0;
            Link *j = body->joint(i);
            if (j) j->q = q;
        }
        ifs.getline(buf, 1024); // skip rest of the line
        body->calcForwardKinematics();

        for (size_t i=0; i<strings.size(); i++){ 
            if (!strings[i].update()){
                std::cerr << "failed to update string state" << std::endl;
            }
        }
        // update body
        updateCharacterPosition(wstate.characterPositions[0], body);
        // update anchors
        for (size_t i=0; i<anchors.size(); i++){
            updateFrame(wstate.collisions[i+1], 
                        anchors[i].position(), Matrix33::Identity());
        }
        // update strings
        size_t n=0, index=0;
        for (size_t i=0; i<strings.size(); i++){
            n += strings[i].polyLine.lines.size();
        }
        wstate.collisions[0].points.length(n);
        for (size_t i=0; i<strings.size(); i++){
            const std::vector<LineSegment>& lines = strings[i].polyLine.lines;
            for (size_t i=0; i<lines.size(); i++){
                updateLineSegment(wstate.collisions[0].points[index++],
                                  lines[i].first, lines[i].second);
            }
        }
        olv->update(wstate);
        wstate.time += 0.005;
        std::cout << wstate.time << " ";
        for (size_t i=0; i<strings.size(); i++){
            std::cout << strings[i].length() << " ";
        }
        std::cout << std::endl;
    }
    return 0;
}
