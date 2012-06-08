#include <QApplication>
#include <iostream>
#include "monitor.h"


#include <rtm/Manager.h>
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include "Simulator.h"
#include "GLmodel.h"
#include "Project.h"
#include "OpenRTMUtil.h"
#include "BodyRTC.h"

using namespace hrp;
using namespace OpenHRP;

hrp::BodyPtr createBody(const std::string& name, const std::string& url,
                        RTC::CorbaNaming *naming)
{
    std::cout << "createBody(" << name << "," << url << ")" << std::endl;
    RTC::Manager& manager = RTC::Manager::instance();
    std::string args = "BodyRTC?instance_name="+name;
    BodyRTCPtr body = (BodyRTC *)manager.createComponent(args.c_str());
    if (!loadBodyFromModelLoader(body, url.c_str(), 
                                 CosNaming::NamingContext::_duplicate(naming->getRootContext()),
                                 true)){
        std::cerr << "failed to load model[" << url << "]" << std::endl;
        manager.deleteComponent(body.get());
        return hrp::BodyPtr();
    }else{
        body->createDataPorts();
        return body;
    }
}

int main( int argc, char **argv )
{
    QApplication app(argc, argv);
    monitor win;
    win.setGeometry( 100, 100, 640, 480 );
    win.show();

    Project prj;
    if (!prj.parse(argv[1])){
        std::cerr << "failed to parse " << argv[1] << std::endl;
        return 1;
    }

    //================= OpenRTM =========================
    RTC::Manager* manager;
    manager = RTC::Manager::init(argc, argv);
    manager->init(argc, argv);
    BodyRTC::moduleInit(manager);
    manager->activateManager();
    manager->runManager(true);

    std::string nameServer = manager->getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){
        comPos = nameServer.length();
    }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(manager->getORB(), nameServer.c_str());

    //==================== Viewer setup ===============
    GLscene *scene = GLscene::getInstance();
    for (std::map<std::string, ModelItem>::iterator it=prj.models().begin();
         it != prj.models().end(); it++){
        OpenHRP::BodyInfo_var binfo
            = loadBodyInfo(it->second.url.c_str(), 
                           CosNaming::NamingContext::_duplicate(naming.getRootContext()));
        GLbody *body = new GLbody(binfo);
        scene->addBody(it->first, body);
    }

    //================= setup World ======================
    BodyFactory factory = boost::bind(createBody, _1, _2, &naming);
    Simulator *simulator = new Simulator;
    simulator->init(prj, factory, scene);
    win.setSimulator(simulator);

    std::cout << "timestep = " << prj.timeStep() << ", total time = " 
              << prj.totalTime() << std::endl;

    app.exec();

    RTC::Manager::instance().shutdown();
}
