#include "command_base.h"
#include "daemon_command.h"
#include "write_command.h"
#include "pause_command.h"

using namespace std;

bool CommandBase::parseTopics(ArgParser& parser, bool* allTopicsOut, std::vector<std::string>* topicsOut)
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

std::shared_ptr<Subcommand> CommandBase::getCommand(ArgParser& parser)
{
    const char* action = parser.getSubcommand("daemon, write, pause, resume");
    if (action == NULL)
        return nullptr;

    std::shared_ptr<Subcommand> cmd;
    if (strcmp(action, "daemon") == 0)
    {
        cmd = std::make_shared<DeamonCommand>();
    }
    else if (action == string("pause"))
    {
        cmd = std::make_shared<PauseCommand>(true);
    }
    else if (action == string("resume"))
    {
        cmd = std::make_shared<PauseCommand>(false);
    }
    else
    {
        cmd = std::make_shared<WriteCommand>();
    }

    if (parser.getArg("help", "h"))
        return cmd;

    if (!cmd->parseArguments(parser))
        return nullptr;

    return cmd;
}
