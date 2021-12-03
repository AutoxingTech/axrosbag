#pragma once

#include <string>
#include <vector>
#include <memory>
#include <deque>
#include <unordered_set>
#include <list>
#include <map>
#include <thread>

#include <ros/ros.h>
#include <ros/time.h>
#include <rosbag/bag.h>
#include <topic_tools/shape_shifter.h>

#include "mutex.h"
#include "resetable_event.h"
#include "axrosbag/TriggerRecord.h"
#include "command_base.h"

using namespace axrosbag;

struct OutgoingMessage
{
    std::string m_topic;
    ros::Time m_time;
    topic_tools::ShapeShifter::ConstPtr m_topicMsg;
    boost::shared_ptr<ros::M_string> m_connectionHeader;
};

struct BagWriter
{
    ros::Time m_startTime;
    TriggerRecord::Request m_req;
    TriggerRecord::Response m_res;
    bool m_readyWrite;
};

class DeamonCommand : public CommandBase
{
public:
    DeamonCommand();
    ~DeamonCommand();

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
    void writeFile();

    bool m_allTopics;
    std::vector<std::string> m_topics;
    float m_timeLimit = 300;
    ros::NodeHandle m_nh;
    ros::Timer m_pollTopicTimer;

    nc::Mutex m_bufferMutex;
    std::deque<OutgoingMessage> m_buffer GUARDED_BY(m_bufferMutex);
    std::map<std::string, OutgoingMessage> m_latchedMsgs GUARDED_BY(m_bufferMutex);

    std::unordered_set<std::string> m_checkTopics;
    std::list<ros::Subscriber> m_subscribers;
    ros::ServiceServer m_triggerServer;

    ResetableEvent m_event;

    BagWriter m_writer;

    std::thread m_writeThread;
};
