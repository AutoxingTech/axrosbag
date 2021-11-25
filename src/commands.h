#pragma once

#include <string>
#include <vector>
#include "nc_argparse.h"
#include <memory>

enum class CompressionType
{
    none,
    bz2,
    lz4
};

enum class RunMode
{
    daemon, // background daemon
    write
};

bool parseTopics(ArgParser& parser, bool* allTopicsOut, std::vector<std::string>* topicsOut);

class DeamonCommand : public Subcommand
{
public:
    void printHelp() override;
    bool parseArguments(ArgParser& parse) override;
    int run() override;

private:
    bool m_allTopics;
    std::vector<std::string> m_topics;
    float m_timeLimit = 300;
};

class WriteCommand : public Subcommand
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
};

std::shared_ptr<Subcommand> getCommand(ArgParser& parser);
