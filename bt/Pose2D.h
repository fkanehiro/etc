#ifndef __POSE2D_H__
#define __POSE2D_H__

#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp/loggers/bt_file_logger.h"
#include "behaviortree_cpp/blackboard/blackboard_local.h"

struct Pose2D
{
    double x,y,theta;
};

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
