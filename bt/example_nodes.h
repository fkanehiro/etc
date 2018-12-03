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

#endif
