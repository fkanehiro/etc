#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "example_nodes.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp/loggers/bt_file_logger.h"
#include "behaviortree_cpp/blackboard/blackboard_local.h"

#ifdef ZMQ_FOUND
#include "behaviortree_cpp/loggers/bt_zmq_publisher.h"
#endif

using namespace BT;

Pose2D robot, ball, bin;

int main()
{
    unsigned int now = (unsigned int)time( 0 );
    srand( now );
    
    BT::BehaviorTreeFactory factory;

    // The state of the door is read/written using these keys of the blackboard.
    auto blackboard = Blackboard::create<BlackboardLocal>();
    //blackboard->set("robot", "250;250;0");
    //blackboard->set("ball", "150;0;0");
    //blackboard->set("bin", "150;150;0");
    robot.x = robot.y = 250; robot.theta = 0;
    ball.x = (rand()*500.0)/RAND_MAX; ball.y = (rand()*500.0)/RAND_MAX; ball.theta = 0;
    bin.x = (rand()*500.0)/RAND_MAX; bin.y = (rand()*500.0)/RAND_MAX; bin.theta = 0;

    // register all the actions into the factory
    Example::RegisterNodes(factory);

    // Important: when the object tree goes out of scope, all the TreeNodes are destroyed
    auto tree = buildTreeFromFile(factory, "example2.1.xml", blackboard);

    // Create some loggers
    //StdCoutLogger logger_cout(tree.root_node);
    //MinitraceLogger logger_minitrace(tree.root_node, "bt_trace.json");
    FileLogger logger_file(tree.root_node, "bt_trace.fbl");
#ifdef ZMQ_FOUND
    PublisherZMQ publisher_zmq(tree.root_node);
#endif

    cv::namedWindow("drawing", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);

    int cnt=0;
    //while (1)
    {
        NodeStatus status = NodeStatus::RUNNING;
        // Keep on ticking until you get either a SUCCESS or FAILURE state
        while( status == NodeStatus::RUNNING)
        {
            std::cout << std::endl << "tick:" << cnt++ << std::endl;
            status = tree.root_node->executeTick();
            //Example::SleepMS(1000);   // optional sleep to avoid "busy loops"

            cv::Mat img = cv::Mat::zeros(500, 500, CV_8UC3);
            // Red
            // 画像，円の中心座標，半径，色，線太さ，種類
            cv::circle(img, cv::Point(robot.x, robot.y), 10, cv::Scalar(0,0,200), -1, CV_AA);
            cv::line(img, cv::Point(robot.x, robot.y),
                     cv::Point(robot.x+50*cos(robot.theta+1.0),
                               robot.y+50*sin(robot.theta+1.0)),
                     cv::Scalar(0,0,200), 1,  CV_AA);
            cv::line(img, cv::Point(robot.x, robot.y),
                     cv::Point(robot.x+50*cos(robot.theta-1.0),
                               robot.y+50*sin(robot.theta-1.0)),
                     cv::Scalar(0,0,200), 1,  CV_AA);
            // Green
            cv::circle(img, cv::Point(ball.x, ball.y), 10, cv::Scalar(0,200,0), -1, CV_AA);
            // Blue
            cv::circle(img, cv::Point(bin.x, bin.y), 10, cv::Scalar(200,0,0), -1, CV_AA);
            cv::imshow("drawing", img);
            cv::waitKey(100);

            if ((rand()*1.0)/RAND_MAX < 0.05){
                ball.x = (rand()*500.0)/RAND_MAX;
                ball.y = (rand()*500.0)/RAND_MAX;
                //std::cout << "ball position is changed" << std::endl;
            }
            
        }
    }
    return 0;
}
