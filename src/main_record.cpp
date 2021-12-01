#include "commands.h"

int printHelp()
{
    printf("Syntax: axrosbag COMMAND [OPTIONS]\n"
           "        axrosbag COMMAND --help\n"
           "\n"
           "COMMAND:\n"
           "  daemon    Record message in background.\n"
           "  write     Write recorded message in to file\n");
    return 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "axrosbag", ros::InitOption::AnonymousName);

    ArgParser parser;
    parser.parse(argc, argv);

    std::shared_ptr<Subcommand> cmd = getCommand(parser);

    // print help
    if (cmd == nullptr)
    {
        if (parser.hasArg("help", "h"))
            return printHelp();

        return 1;
    }

    // print help of subcommand
    if (parser.hasArg("help", "h"))
    {
        cmd->printHelp();
        return 0;
    }

    return cmd->run();
}
