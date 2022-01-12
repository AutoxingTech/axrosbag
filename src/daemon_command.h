#pragma once

#include <string>
#include <vector>
#include <memory>
#include <deque>
#include <unordered_set>
#include <list>
#include <map>
#include <set>
#include <thread>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "mutex.h"
#include "axrosbag/TriggerRecord.h"
#include "axrosbag/PauseResume.h"
#include "command_base.h"
#include "common_types.h"

using namespace axrosbag;

class DeamonCommand : public CommandBase
{
public:
    DeamonCommand();
    ~DeamonCommand();

    void printHelp() override;
    bool parseArguments(ArgParser& parse) override;
    int run() override;

private:
    void subscribeTopic(std::string& topic);
    bool writeServiceCallback(TriggerRecord::Request& req, TriggerRecord::Response& res);
    bool pauseResumeServiceCallback(PauseResume::Request& req, PauseResume::Response& res);
    void topicCallback(const std::string& topic, const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event);

    void pollTopicsTimer(const ros::TimerEvent& e);
    void removeMessageTimer(const ros::TimerEvent& e);

private:
    // parameters
    bool m_allTopics;
    std::vector<std::string> m_topics;
    float m_timeLimit = 300;

    // for service
    ros::CallbackQueue m_callbackQueue;
    ros::AsyncSpinner m_asyncSpiner;
    ros::NodeHandle m_asyncHandle;

    ros::NodeHandle m_nh;
    ros::Timer m_pollTopicTimer;
    ros::Timer m_removeMessageTimer;

    nc::Mutex m_bufferMutex;
    std::deque<OutgoingMessage> m_buffer GUARDED_BY(m_bufferMutex);
    std::map<std::pair<std::string, std::string>, OutgoingMessage> m_latchedMsgs GUARDED_BY(m_bufferMutex);

    std::unordered_set<std::string> m_checkTopics;
    std::list<ros::Subscriber> m_subscribers;
    ros::ServiceServer m_triggerServer;

    nc::Mutex m_pauseMutex;
    bool m_pauseAllTopics GUARDED_BY(m_pauseMutex) = false;
    std::set<std::string> m_pausedTopics GUARDED_BY(m_pauseMutex);
    ros::ServiceServer m_pauseServer;
    ros::ServiceServer m_resumeServer;
};
