#pragma once

#include <string>
#include <vector>
#include <ros/ros.h>
#include "command_base.h"
#include "common_types.h"

class PauseCommand : public CommandBase
{
public:
    PauseCommand(bool isPause) : m_nh("~") { m_isPause = isPause; }

    void printHelp() override;
    bool parseArguments(ArgParser& parse) override;
    int run() override;

private:
    bool m_isPause;
    std::vector<std::string> m_topics;
    bool m_allTopics;
    ros::NodeHandle m_nh;
};
