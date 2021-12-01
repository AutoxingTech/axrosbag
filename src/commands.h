#pragma once

#include <string>
#include <vector>
#include <memory>
#include <deque>
#include <unordered_set>
#include <list>
#include <mutex>
#include <map>

#include <ros/ros.h>
#include <ros/time.h>
#include <rosbag/bag.h>
#include <topic_tools/shape_shifter.h>

#include "nc_argparse.h"
#include "axrosbag/TriggerRecord.h"

using namespace axrosbag;

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

struct OutgoingMessage
{
    std::string m_topic;
    ros::Time m_time;
    topic_tools::ShapeShifter::ConstPtr m_topicMsg;
    boost::shared_ptr<ros::M_string> m_connectionHeader;
};

class DeamonCommand : public Subcommand
{
public:
    void printHelp() override;
    bool parseArguments(ArgParser& parse) override;
    int run() override;

private:
    void pollTopics();
    void subscribeTopic(std::string& topic);
    bool triggerRecordCB(TriggerRecord::Request& req, TriggerRecord::Response& res);
    void topicCB(const std::string& topic, const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event);
    bool checkQueue(ros::Time& time);
    bool writeTopic(rosbag::Bag& bag, const OutgoingMessage& msg, TriggerRecord::Request& req,
                    TriggerRecord::Response& res);

    bool m_allTopics;
    std::vector<std::string> m_topics;
    float m_timeLimit = 300;
    ros::NodeHandle m_nh;
    ros::Timer m_pollTopicTimer;
    std::deque<OutgoingMessage> m_buffer;
    std::map<std::string, OutgoingMessage> m_latchedMsgs;
    std::unordered_set<std::string> m_checkTopics;
    std::list<ros::Subscriber> m_subscribers;
    ros::ServiceServer m_triggerServer;
    std::mutex m_mutex;
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
    ros::NodeHandle m_nh;
};

std::shared_ptr<Subcommand> getCommand(ArgParser& parser);
