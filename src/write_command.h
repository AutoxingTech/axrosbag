#pragma once

#include <string>
#include <vector>
#include <ros/ros.h>
#include <ros/time.h>
#include <rosbag/bag.h>
#include <topic_tools/shape_shifter.h>
#include "mutex.h"
#include "resetable_event.h"
#include "command_base.h"

#include "axrosbag/TriggerRecord.h"

using namespace axrosbag;

enum class CompressionType
{
    none,
    bz2,
    lz4
};

class WriteCommand : public CommandBase
{
public:
    void printHelp() override;
    bool parseArguments(ArgParser& parse) override;
    int run() override;

private:
    std::string m_filename;
    std::vector<std::string> m_topics;
    bool m_allTopics;
    CompressionType m_compressType = CompressionType::none;
    ros::NodeHandle m_nh;
};

std::shared_ptr<Subcommand> getCommand(ArgParser& parser);
