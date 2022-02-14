#include "daemon_command.h"
#include "bag_writer.h"

using namespace std;
using namespace nc;

DeamonCommand::DeamonCommand() : m_nh("~"), m_asyncSpiner(1, &m_callbackQueue), m_asyncHandle("~")
{
    m_asyncHandle.setCallbackQueue(&m_callbackQueue);
    m_asyncSpiner.start();
}

DeamonCommand::~DeamonCommand()
{
    for (auto& sub : m_subscribers)
    {
        sub.shutdown();
    }
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

void DeamonCommand::removeMessageTimer(const ros::TimerEvent& /* e */)
{
    ros::Time now = ros::Time::now();

    nc::LockGuard lg(m_bufferMutex);
    if (!m_buffer.empty() && now < m_buffer.back().recvTime)
    {
        ROS_WARN("Time has gone backwards, clearing buffer for this topic.");
        m_buffer.clear();
        m_buffer.shrink_to_fit();
        return;
    }

    size_t i = 0;
    for (; i < m_buffer.size(); i++)
    {
        const ros::Time& recvTime = m_buffer[i].recvTime;
        float dt = (now - recvTime).toSec();
        if (dt < m_timeLimit)
        {
            break;
        }

        // keep latched messages
        const auto& connectionHeader = m_buffer[i].connectionHeader;
        if (connectionHeader)
        {
            auto it = connectionHeader->find("latching");
            if ((it != connectionHeader->end()) && (it->second == "1"))
            {
                auto it2 = connectionHeader->find("callerid");
                if (it2 != connectionHeader->end())
                {
                    m_latchedMsgs[{m_buffer[i].topic, it2->second}] = m_buffer[i];
                }
            }
        }
    }

    m_buffer.erase(m_buffer.begin(), m_buffer.begin() + i);
    ROS_DEBUG("Messages in queue %d", (int)m_buffer.size());
}

void DeamonCommand::topicCallback(const std::string& topic, float frequency,
                                  const ros::MessageEvent<topic_tools::ShapeShifter const>& msgEvent)
{
    bool paused = false;
    {
        LockGuard lg(m_pauseMutex);
        if (m_pauseAllTopics)
        {
            paused = true;
        }
        else
        {
            paused = m_pausedTopics.find(topic) != m_pausedTopics.end();
        }
    }

    if (!paused)
    {
        ros::Time recvTime = ros::Time::now();
        OutgoingMessage msg{topic, recvTime, msgEvent.getMessage(), msgEvent.getConnectionHeaderPtr()};

        if (frequency > 0.f)
        {
            auto res = m_lowerFrequencyTopics.insert(make_pair(topic, recvTime));
            if (res.second)
            {
                return; // map init
            }

            if ((recvTime - m_lowerFrequencyTopics[topic]).toSec() < 1.0 / frequency)
            {
                return; // skip some data
            }
            else
            {
                m_lowerFrequencyTopics[topic] = recvTime; // update map
            }
        }

        nc::LockGuard lg(m_bufferMutex);
        m_buffer.push_back(msg);
    }
}

void DeamonCommand::subscribeTopic(std::string& topic, float frequency)
{
    ROS_INFO("Subscribing to %s", topic.c_str());

    ros::SubscribeOptions ops;
    ops.topic = topic;
    ops.queue_size = 100;
    ops.md5sum = ros::message_traits::md5sum<topic_tools::ShapeShifter>();
    ops.datatype = ros::message_traits::datatype<topic_tools::ShapeShifter>();
    ops.helper =
        boost::make_shared<ros::SubscriptionCallbackHelperT<const ros::MessageEvent<topic_tools::ShapeShifter const>&>>(
            [this, topic, frequency](const ros::MessageEvent<topic_tools::ShapeShifter const>& ev) {
                topicCallback(topic, frequency, ev);
            });
    m_subscribers.push_back(m_nh.subscribe(ops));
}

void DeamonCommand::pollTopicsTimer(const ros::TimerEvent& /* e */)
{
    ros::master::V_TopicInfo topics;
    if (ros::master::getTopics(topics))
    {
        for (auto& t : topics)
        {
            auto res = m_checkTopics.insert(t.name);
            if (res.second)
            {
                subscribeTopic(t.name);
            }
        }
    }
    else
    {
        ROS_WARN_STREAM_THROTTLE(5, "Failed to get topics from the ROS master");
    }
    ros::master::V_TopicInfo().swap(topics);
}

bool DeamonCommand::writeServiceCallback(TriggerRecord::Request& req, TriggerRecord::Response& res)
{
    std::unique_ptr<BagWriter> writer;
    {
        LockGuard lg(m_bufferMutex);
        writer = std::make_unique<BagWriter>(m_buffer, m_latchedMsgs);
    }

    if (!writer->writeIntoFile(req.filename, (CompressionType)req.compression_type))
    {
        res.message = writer->errorMessage();
        res.success = false;
        return true;
    }

    res.success = true;
    return true;
}

bool DeamonCommand::pauseResumeServiceCallback(PauseResume::Request& req, PauseResume::Response& res)
{
    LockGuard lg(m_pauseMutex);
    if (req.is_pause)
    {
        if (req.all_topics)
        {
            m_pauseAllTopics = true;
            m_pausedTopics.clear();
        }
        else
        {
            m_pauseAllTopics = false;
            for (const auto& topic : req.topics)
            {
                m_pausedTopics.insert(topic);
            }
        }
    }
    else
    {
        m_pauseAllTopics = false;
        if (req.all_topics)
        {
            m_pausedTopics.clear();
        }
        else
        {
            for (const auto& topic : req.topics)
            {
                m_pausedTopics.erase(topic);
            }
        }
    }

    res.all_paused = m_pauseAllTopics;
    res.paused_topics.resize(m_pausedTopics.size());
    std::copy(m_pausedTopics.begin(), m_pausedTopics.end(), res.paused_topics.begin());
    res.success = true;
    return true;
}

int DeamonCommand::run()
{
    if (!m_nh.ok())
        return 1;

    m_triggerServer = m_asyncHandle.advertiseService("/axrosbag/write", &DeamonCommand::writeServiceCallback, this);
    m_pauseServer = m_asyncHandle.advertiseService("/axrosbag/pause", &DeamonCommand::pauseResumeServiceCallback, this);
    m_removeMessageTimer = m_nh.createTimer(ros::Duration(1), &DeamonCommand::removeMessageTimer, this);

    if (m_allTopics)
    {
        printf("recoding all topics\n");
        m_pollTopicTimer = m_nh.createTimer(ros::Duration(1.0), &DeamonCommand::pollTopicsTimer, this);
    }
    else
    {
        printf("recoding topics:\n");
        for (auto& topic : m_topics)
        {
            int nPos = topic.find("@");
            if (nPos != -1)
            {
                std::string topicName = topic.substr(0, nPos);
                std::string topicFrequency = topic.substr(nPos + 1, topic.length() - topicName.length() - 3);
                float frequency = atof(topicFrequency.c_str());
                auto res = m_checkTopics.insert(topicName);
                if (res.second)
                {
                    printf("\t%s\n", topic.c_str());
                    subscribeTopic(topicName, frequency);
                }
            }
            else
            {
                auto res = m_checkTopics.insert(topic);
                if (res.second)
                {
                    printf("\t%s\n", topic.c_str());
                    subscribeTopic(topic);
                }
            }
        }
    }

    ros::spin();

    return 0;
}
