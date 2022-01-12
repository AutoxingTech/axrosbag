#include "pause_command.h"
#include "axrosbag/PauseResume.h"

using namespace std;
using namespace axrosbag;

void PauseCommand::printHelp()
{
    printf("Syntax: axrosbag pause/resume [OPTIONS] <TOPIC1> <TOPIC2> ...\n"
           "\n"
           "Options:\n"
           "\n"
           "--all,-a      All topics\n");
}

bool PauseCommand::parseArguments(ArgParser& parser)
{
    return parseTopics(parser, &m_allTopics, &m_topics);
}

int PauseCommand::run()
{
    ros::ServiceClient client = m_nh.serviceClient<PauseResume>("/axrosbag/pause");
    if (!client.exists())
    {
        ROS_ERROR("Service pause does not exist. Is record running in this namespace?");
        return 1;
    }

    PauseResumeRequest req;
    req.is_pause = m_isPause;
    req.all_topics = m_allTopics;
    req.topics = m_topics;

    PauseResumeResponse res;
    if (!client.call(req, res))
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }
    if (!res.success)
    {
        ROS_ERROR("Failed to call axrosbag pause");
        return 1;
    }

    if (res.all_paused)
    {
        printf("All topics are paused.\n");
    }
    else if (!res.paused_topics.empty())
    {
        printf("Paused Topics: \n");
        for (const auto& topic : res.paused_topics)
        {
            printf("%s\n", topic.c_str());
        }
    }
    else
    {
        printf("No paused Topics.\n");
    }

    return 0;
}
