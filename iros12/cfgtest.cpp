#include <Model/HumanoidBodyUtil.h>
//#define EXACT
#ifdef EXACT
#include "myCfgSetter3exactCOG.h"
#else
#include "myCfgSetter3.h"
#endif
#include "RobotUtil.h"
#include "problem.h"
//#define DISPLAY

using namespace motion_generator;
using namespace hrp;
using namespace PathEngine;

int main(int argc, char *argv[])
{
    const char *robotURL = NULL;
    Vector3 goalP, goalRpy;

    for(int i = 1 ; i < argc; i++){
        if (strcmp(argv[i], "-robot") == 0){
            robotURL = argv[++i];
        }else if (strcmp(argv[i], "-goalpos") == 0){
            for (int j=0; j<3; j++) {
                goalP(j) = atof(argv[++i]);
            }
            for (int j=0; j<3; j++) {
                goalRpy(j) = atof(argv[++i]);
            }
        }
    }

    HumanoidBodyPtr robot = HumanoidBodyPtr(new HumanoidBody());
    loadHumanoidBodyFromModelLoader(robot, robotURL, argc, argv, true);
    // set halfconf
    setHalfConf(robot);

    int armDof = 6;
    problem prob(4+armDof*2);
    prob.addRobot("robot", robotURL, robot);
#ifdef DISPLAY
    prob.initOLV(argc, argv);
#endif

    myCfgSetter3 setter(robot, goalP);

    int dim = 7;
    ConfigurationSpace CSforGoal(dim); 
    CSforGoal.bounds(0,  0.26, 0.705); // body z
    CSforGoal.bounds(1, -0.5, 0.5); // body roll
    CSforGoal.bounds(2, -0.0, 0.5); // body pitch
    CSforGoal.bounds(3, -0.5, 0.5); // body yaw
    CSforGoal.bounds(4, -M_PI/2, M_PI/2);   // hand roll
    CSforGoal.bounds(5, -M_PI/2, M_PI/2); // hand pitch
    CSforGoal.bounds(6, -M_PI, M_PI);   // hand yaw

    TimeMeasure tm;
    int success=0, n = 10000;
    for (int i=0; i<n; i++){
        Configuration cfg = CSforGoal.random();

        //std::cout << "cfg = " << cfg << std::endl;
        tm.begin();
        bool ret = setter.set(NULL, cfg);
        tm.end();
        if (ret){
            success++;
#ifdef DISPLAY
            prob.updateOLV();
#endif
        }
        //sleep(1);
    }
    std::cout << "average:" << tm.averageTime()*1e3 << "[ms]" << std::endl;
    std::cout << "success:" << success << "/" << n << std::endl;

    return 0;
}
