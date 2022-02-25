#include "write_command.h"
#include "axrosbag/TriggerRecord.h"

using namespace axrosbag;
using namespace std;

void WriteCommand::printHelp()
{
    printf("Syntax: axrosbag write -f FILENAME [OPTIONS] <TOPIC1> <TOPIC2> ...\n"
           "\n"
           "Options:\n"
           "\n"
           "--all,-a      All topics\n");
}

bool WriteCommand::parseArguments(ArgParser& parser)
{
    if (!parseTopics(parser, &m_allTopics, &m_topics))
        return false;

    // -f
    const char* filename = parser.getArg("f");
    if (filename == NULL)
    {
        printf("error: please provide filename\n");
        return false;
    }

    m_filename = filename;

    // --lz4 or --bz2
    if (parser.hasArg("bz2") && parser.hasArg("lz4"))
    {
        printf("error: must choose from eithor --bz2 or --lz4\n");
        return false;
    }
    else if (parser.hasArg("bz2"))
    {
        m_compressType = CompressionType::bz2;
    }
    else
    {
        m_compressType = CompressionType::lz4;
    }

    // --duration
    if (!parser.hasArg("duration"))
    {
        printf("error: please provide --duration 60s or --duration 1m (max 300s / 5m)\n");
        return false;
    }
    std::string duration_limit = parser.getArg("duration");

    if (CommandBase::_strEndsWith(duration_limit, "s"))
    {
        int nPos = duration_limit.find("s");
        std::string dt = duration_limit.substr(0, nPos);
        m_durationLimit = atoi(dt.c_str());
    }
    else if (CommandBase::_strEndsWith(duration_limit, "m"))
    {
        int nPos = duration_limit.find("m");
        std::string dt = duration_limit.substr(0, nPos);
        m_durationLimit = atoi(dt.c_str()) * 60; // unit: seconds
    }
    else
    {
        m_durationLimit = atoi(duration_limit.c_str());
    }

    if (m_durationLimit == 0)
    {
        ROS_ERROR("write duration limit must not be 0: %s", duration_limit.c_str());
        return false;
    }

    return true;
}

int WriteCommand::run()
{
    ros::ServiceClient client = m_nh.serviceClient<TriggerRecord>("/axrosbag/write");
    if (!client.exists())
    {
        ROS_ERROR("Service write does not exist. Is record running in this namespace?");
        return 1;
    }

    TriggerRecordRequest req;
    req.compression_type = (int)(m_compressType);
    req.duration_limit = m_durationLimit;

    boost::filesystem::path p(boost::filesystem::system_complete(m_filename));
    req.filename = p.string();

    TriggerRecordResponse res;
    if (!client.call(req, res))
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }
    if (!res.success)
    {
        ROS_ERROR("%s", res.message.c_str());
        return 1;
    }

    printf("writing file %s\n", p.c_str());
    return 0;
}
