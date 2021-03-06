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

    const char* filename = parser.getArg("f");
    if (filename == NULL)
    {
        printf("error: please provide filename\n");
        return false;
    }

    m_filename = filename;

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
