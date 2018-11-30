#ifndef __EXAMPLE_NODES_H__
#define __EXAMPLE_NODES_H__

#include "behaviortree_cpp/bt_factory.h"

using namespace BT;
namespace Example
{
inline void SleepMS(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

BT::NodeStatus AskForHelp(TreeNode& self);
BT::NodeStatus BallFound(TreeNode& self);
BT::NodeStatus BallClose(TreeNode& self);
BT::NodeStatus BallGrasped(TreeNode& self);
BT::NodeStatus BinFound(TreeNode& self);
BT::NodeStatus BinClose(TreeNode& self);
BT::NodeStatus BallPlaced(TreeNode& self);
BT::NodeStatus FindBall(TreeNode& self);
BT::NodeStatus ApproachBall(TreeNode& self);
BT::NodeStatus GraspBall(TreeNode& self);
BT::NodeStatus FindBin(TreeNode& self);
BT::NodeStatus ApproachBin(TreeNode& self);
BT::NodeStatus PlaceBall(TreeNode& self);

void RegisterNodes(BT::BehaviorTreeFactory& factory);
};

struct Pose2D
{
    double x,y,theta;
};

#if 0
namespace BT
{
// This template specialization is needed if you want
// to AUTOMATICALLY convert a NodeParameter into a Pose2D
template <>
Pose2D convertFromString(const StringView& key)
{
    // Three real numbers separated by semicolons.
    // You may use <boost/algorithm/string/split.hpp>  if you prefer
    auto parts = BT::splitString(key, ';');
    if( parts.size() != 3)
    {
        throw std::runtime_error("invalid input)");
    }
    else{
        Pose2D output;
        output.x     = convertFromString<double>( parts[0] );
        output.y     = convertFromString<double>( parts[1] );
        output.theta = convertFromString<double>( parts[2] );
        return output;
    }
}
};    
#endif
#endif
