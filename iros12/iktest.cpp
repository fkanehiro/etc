#include <Model/HumanoidBodyUtil.h>
#include <hrpModel/JointPath.h>
#include <hrpModel/Link.h>
#include <hrpUtil/MatrixSolvers.h>
#include "problem.h"
#include <Math/Physics.h>
#include "RobotUtil.h"

using namespace motion_generator;
using namespace hrp;

static Vector3 _omegaFromRot(const Matrix33& r)
{
    const double k = -0.5;
    return Vector3( (r(1,2)-r(2,1)) * k, (r(2,0)-r(0,2)) * k, (r(0,1)-r(1,0)) * k );
};

bool calcInverseKinematics(JointPathPtr jpath, const Vector3& end_p, const Matrix33& end_R, problem &prob)
{
    static const int MAX_IK_ITERATION = 50;
    static const double LAMBDA = 0.9;
    static const double maxIKErrorSqr = 1.0e-6 * 1.0e-6;
    double prevError;

    const int n = jpath->numJoints();

    Link* target = jpath->endLink();

    dmatrix J(6, n);
    dvector dq(n);
    dvector v(6);

    double errsqr = maxIKErrorSqr * 100.0;

    for(int i=0; i < MAX_IK_ITERATION; i++){
	
        jpath->calcJacobian(J);
	
        Vector3 dp(end_p - target->p);
        Vector3 omega(target->R * omegaFromRot(target->R.transpose() * end_R));

        const double errsqr = dp.dot(dp) + omega.dot(omega);
        std::cout << i << ":" << errsqr << std::endl;
        if (i==0) prevError = errsqr;
        if (errsqr > prevError) return false;
        if(errsqr < maxIKErrorSqr){
            return true;
        }

        v << dp, omega;
		
        if(n == 6){ 
            solveLinearEquationLU(J, v, dq);
        } else {
            solveLinearEquationSVD(J, v, dq);  // dq = pseudoInverse(J) * v
        }
		
        for(int j=0; j < n; ++j){
            jpath->joint(j)->q += LAMBDA * dq(j);
        }

        jpath->calcForwardKinematics();
        prob.updateOLV();
        prevError = errsqr;
    }

    return false;
}


int main(int argc, char *argv[])
{
    srand((unsigned)time(NULL));

    const char *robotURL = NULL;
    const char *goalURL = NULL;
    Vector3 p, rpy;
    // goal position
    Vector3 goalP, goalRpy;
    for(int i = 1 ; i < argc; i++){
        if (strcmp(argv[i], "-robot") == 0){
            robotURL = argv[++i];
        }else if (strcmp(argv[i], "-goal") == 0){
            goalURL = argv[++i];
        }else if (strcmp(argv[i], "-goalpos") == 0){
            for (int j=0; j<3; j++) {
                goalP(j) = atof(argv[++i]);
            }
            for (int j=0; j<3; j++) {
                goalRpy(j) = atof(argv[++i]);
            }
        }
    }
    if (robotURL == NULL){
        std::cerr << "please specify URL of VRML model by -robot option"
                  << std::endl;
        return 1;
    }


    HumanoidBodyPtr robot = HumanoidBodyPtr(new HumanoidBody());
    loadHumanoidBodyFromModelLoader(robot, robotURL, argc, argv, true);

    problem prob(0);
    prob.addRobot("robot", robotURL, robot);

    BodyPtr goal;
    if (goalURL){
        goal = prob.addGoal("goal", goalURL);
    }

    // This must be called after all bodies are added
    prob.initOLV(argc, argv);

    prob.initPlanner();

    // set halfconf
    setHalfConf(robot);
    robot->calcForwardKinematics();

    Matrix33 goalR(rotFromRpy(goalRpy));
    if (goal){
        goal->rootLink()->p = goalP;
        goal->rootLink()->R = goalR;
        goal->calcForwardKinematics();
    }

    prob.updateOLV();

#if 0
    // numerical solution
    JointPathPtr jpath = JointPathPtr(new JointPath(robot->chestLink,
                                                    robot->wristLink[RIGHT]));
    std::cout << calcInverseKinematics(jpath, goalP, goalR, prob) << std::endl;
#else
    // analytical solution
    
    JointPathPtr jpath;
    if (robot->modelName() == "HRP2SH"){
        jpath = robot->getJointPath(robot->chestLink,
                                    robot->wristLink[RIGHT]->parent);
    }else{
        jpath = robot->getJointPath(robot->chestLink,
                                    robot->wristLink[RIGHT]);
    }
    std::cout << "joint path:" << jpath->baseLink()->name << "->"
              << jpath->endLink()->name << std::endl;
    std::cout << "AnalyticalIK is " << (jpath->hasAnalyticalIK() ? "used" : "not used") << ", numJoints = " << jpath->numJoints() << std::endl;
    for (int i=0;i<10;i++){
        std::cout << i << std::endl;
        for (int j=0; j<jpath->numJoints(); j++){
            Link *l = jpath->joint(j);
            l->q = l->llimit + (rand()*(l->ulimit - l->llimit))/RAND_MAX;
        }
        jpath->calcForwardKinematics();
        goalP = jpath->endLink()->p;
        goalR = jpath->endLink()->R;
        robot->calcForwardKinematics();
        prob.updateOLV();
        if (!jpath->calcInverseKinematics(goalP, goalR)){
            break;
        }
    }
#endif

    return 0;
}
