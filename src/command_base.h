#pragma once

#include "nc_argparse.h"
#include <memory>
#include <vector>

class CommandBase : public Subcommand
{
public:
    static std::shared_ptr<Subcommand> getCommand(ArgParser& parser);

    static bool parseTopics(ArgParser& parser, bool* allTopicsOut, std::vector<std::string>* topicsOut);

    static bool _strEndsWith(const std::string& fullString, const std::string& ending);
};
