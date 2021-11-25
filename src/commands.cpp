#include "commands.h"

using namespace std;

bool parseTopics(ArgParser& parser, bool* allTopicsOut, std::vector<std::string>* topicsOut)
{
    *allTopicsOut = parser.hasArg("all", "a");
    topicsOut->clear();

    if (*allTopicsOut && parser.getPositionalArgNumber() != 0)
    {
        printf("error: if --all exists, topics list must not be provided.\n");
        return false;
    }

    for (size_t i = 0; i < parser.getPositionalArgNumber(); i++)
    {
        topicsOut->push_back(parser.getPositionalArgByIndex(i));
    }

    return true;
}

void DeamonCommand::printHelp()
{
    printf("Syntax: axrosbag daemon [OPTIONS] <TOPIC1> <TOPIC2> ...\n"
           "\n"
           "Options:\n"
           "\n"
           "  --all,-a      All topics.\n"
           "  --limit       Time limit in seconds. Default to 300.\n");
}

bool DeamonCommand::parseArguments(ArgParser& parser)
{
    if (!parseTopics(parser, &m_allTopics, &m_topics))
        return false;

    parser.setDefault("limit", "300");
    m_timeLimit = atof(parser.getArg("limit"));

    return true;
}

int DeamonCommand::run()
{
    if (m_allTopics)
    {
        printf("recoding all topics\n");
    }
    else
    {
        printf("recoding topics:\n");
        for (auto& topic : m_topics)
        {
            printf("\t%s\n", topic.c_str());
        }
    }
    return 0;
}

////////////////////////////////////////////////////////////////////////////////////

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
        printf("error: must choose from eithor --bz4 or --lz2\n");
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
    printf("writing file %s\n", m_filename.c_str());
    return 0;
}

////////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Subcommand> getCommand(ArgParser& parser)
{
    const char* action = parser.getSubcommand("daemon, write");
    if (action == NULL)
        return nullptr;

    std::shared_ptr<Subcommand> cmd;
    if (strcmp(action, "daemon") == 0)
    {
        cmd = std::make_shared<DeamonCommand>();
    }
    else
    {
        cmd = std::make_shared<WriteCommand>();
    }

    if (!cmd->parseArguments(parser))
        return nullptr;

    return cmd;
}
