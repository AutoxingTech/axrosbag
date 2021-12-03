#pragma once

#include <ros/ros.h>
#include <ros/time.h>
#include <topic_tools/shape_shifter.h>

struct OutgoingMessage
{
    std::string topic;
    ros::Time recvTime;
    topic_tools::ShapeShifter::ConstPtr topicMsg;
    boost::shared_ptr<ros::M_string> connectionHeader;
};

enum class CompressionType
{
    none = 0,
    bz2 = 1,
    lz4 = 2
};