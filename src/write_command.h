#pragma once

#include <string>
#include <vector>
#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include "command_base.h"
#include "common_types.h"

class WriteCommand : public CommandBase
{
public:
    WriteCommand() : m_nh("~") {}

    void printHelp() override;
    bool parseArguments(ArgParser& parse) override;
    int run() override;

private:
    std::string m_filename;
    std::vector<std::string> m_topics;
    bool m_allTopics;
    CompressionType m_compressType = CompressionType::none;
    ros::NodeHandle m_nh;
    int m_durationLimit;
};
