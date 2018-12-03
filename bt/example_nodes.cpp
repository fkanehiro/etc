#include "behaviortree_cpp/bt_factory.h"
#include "Pose2D.h"

using namespace BT;

namespace Example {
    
double direction(const Pose2D& self, const Pose2D& target)
{
    auto dx = target.x - self.x;
    auto dy = target.y - self.y;
    auto th = atan2(dy, dx);
    auto dth = th - self.theta;
    while (dth >  M_PI) dth -= 2*M_PI;
    while (dth < -M_PI) dth += 2*M_PI;
    return dth;
}

double distance(const Pose2D& obj1, const Pose2D& obj2)
{
    auto dx = obj1.x - obj2.x;
    auto dy = obj1.y - obj2.y;
    return sqrt(dx*dx+dy*dy);
}

NodeStatus BallFound(TreeNode& self)
{
    std::cout << "BallFound" << std::endl;
    Pose2D robot, ball;
    self.blackboard()->get("robot",robot);
    self.blackboard()->get("ball",ball);
    auto dth = direction(robot, ball);
    auto d = distance(robot, ball);
    if (fabs(dth) > 1.0 && d > 10){
        return NodeStatus::FAILURE;
    }else{
        return NodeStatus::SUCCESS;
    }
}

#if 0
NodeStatus BallClose(TreeNode& self)
{
    std::cout << "BallClose" << std::endl;
    Pose2D robot, ball;
    self.blackboard()->get("robot",robot);
    self.blackboard()->get("ball",ball);
    auto d = distance(ball, robot);
    //std::cout << "distance = " << distance << std::endl;
    if (d < 10){
        return NodeStatus::SUCCESS;
    }else{
        return NodeStatus::FAILURE;
    }
}
#else
class BallClose: public BT::ActionNodeBase
{
public:
    BallClose(const std::string& name, const NodeParameters& params)
        : BT::ActionNodeBase(name, params) {}

    static const NodeParameters& requiredNodeParameters(){
        static NodeParameters params = {{"thd", "10"}};
        return params;
    }
        
    NodeStatus tick() override {
        std::cout << "BallClose" << std::endl;
        Pose2D robot, ball;
        blackboard()->get("robot",robot);
        blackboard()->get("ball",ball);
        auto d = distance(ball, robot);
        //std::cout << "distance = " << distance << std::endl;
        double thd;
        if (!getParam<double>("thd", thd)){
            auto default_thd = requiredNodeParameters().at("thd");
            thd = BT::convertFromString<double>(default_thd);
        }
        std::cout << thd << std::endl;
        if (d < thd){
            return NodeStatus::SUCCESS;
        }else{
            return NodeStatus::FAILURE;
        }
    }

    virtual void halt() override {}
};
#endif

NodeStatus BallGrasped(TreeNode& self)
{
    std::cout << "BallGrasped" << std::endl;
    return NodeStatus::SUCCESS;
}

NodeStatus BinFound(TreeNode& self)
{
    std::cout << "BinFound" << std::endl;
    Pose2D robot, bin;
    self.blackboard()->get("robot",robot);
    self.blackboard()->get("bin",bin);
    auto dth = direction(robot, bin);
    if (fabs(dth) > 1.0){
        return NodeStatus::FAILURE;
    }else{
        return NodeStatus::SUCCESS;
    }
}

NodeStatus BinClose(TreeNode& self)
{
    std::cout << "BinClose" << std::endl;
    Pose2D robot, bin;
    self.blackboard()->get("robot",robot);
    self.blackboard()->get("bin",bin);
    auto d = distance(robot, bin);
    if (d < 10){
        return NodeStatus::SUCCESS;
    }else{
        return NodeStatus::FAILURE;
    }
}

NodeStatus BallPlaced(TreeNode& self)
{
    std::cout << "BallPlaced" << std::endl;
    return NodeStatus::SUCCESS;
}

NodeStatus AskForHelp(TreeNode& self)
{
    std::cout << "AskForHelp" << std::endl;
    return NodeStatus::SUCCESS;
}

NodeStatus FindBall(TreeNode& self)
{
    std::cout << "FindBall" << std::endl;
    Pose2D robot;
    self.blackboard()->get("robot",robot);
    robot.theta += 0.5;
    self.blackboard()->set("robot",robot);
    return NodeStatus::RUNNING;
}

NodeStatus ApproachBall(TreeNode& self)
{
    std::cout << "ApproachBall" << std::endl;
    Pose2D robot, ball;
    self.blackboard()->get("robot",robot);
    self.blackboard()->get("ball", ball);
    auto dx = robot.x - ball.x;
    auto dy = robot.y - ball.y;
    auto distance = sqrt(dx*dx+dy*dy);
    if (distance > 10){
        robot.x -= dx*10/distance;
        robot.y -= dy*10/distance;
        self.blackboard()->set("robot",robot);
        return NodeStatus::RUNNING;
    }else{
        return NodeStatus::SUCCESS;
    }
}

NodeStatus GraspBall(TreeNode& self)
{
    std::cout << "GraspBall" << std::endl;
    return NodeStatus::SUCCESS;
}

NodeStatus FindBin(TreeNode& self)
{
    std::cout << "FindBin" << std::endl;
    Pose2D robot;
    self.blackboard()->get("robot",robot);
    robot.theta += 0.5;
    self.blackboard()->set("robot",robot);
    return NodeStatus::RUNNING;
}

NodeStatus ApproachBin(TreeNode& self)
{
    std::cout << "ApproachBin" << std::endl;
    Pose2D robot, ball, bin;
    self.blackboard()->get("robot",robot);
    self.blackboard()->get("ball",ball);
    self.blackboard()->get("bin",bin);
    auto dx = robot.x - bin.x;
    auto dy = robot.y - bin.y;
    auto distance = sqrt(dx*dx+dy*dy);
    robot.x -= dx*10/distance;
    robot.y -= dy*10/distance;
    ball.x -= dx*10/distance;
    ball.y -= dy*10/distance;
    self.blackboard()->set("robot",robot);
    self.blackboard()->set("ball",ball);
    return NodeStatus::RUNNING;
}

NodeStatus PlaceBall(TreeNode& self)
{
    std::cout << "PlaceBall" << std::endl;
    return NodeStatus::SUCCESS;
}

BT_REGISTER_NODES(factory)    
{
    factory.registerSimpleCondition("BallFound", BallFound);
#if 0
    factory.registerSimpleCondition("BallClose", BallClose);
#else
    factory.registerNodeType<BallClose>("BallClose");
#endif
    factory.registerSimpleCondition("BallGrasped", BallGrasped);
    factory.registerSimpleCondition("BinFound", BinFound);
    factory.registerSimpleCondition("BinClose", BinClose);
    factory.registerSimpleCondition("BallPlaced", BallPlaced);

    factory.registerSimpleAction("AskForHelp", AskForHelp);
    factory.registerSimpleAction("FindBall", FindBall);
    factory.registerSimpleAction("ApproachBall", ApproachBall);
    factory.registerSimpleAction("GraspBall", GraspBall);
    factory.registerSimpleAction("FindBin", FindBin);
    factory.registerSimpleAction("ApproachBin", ApproachBin);
    factory.registerSimpleAction("PlaceBall", PlaceBall);
}
};
