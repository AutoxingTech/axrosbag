#include "commands.h"

using namespace std;
using namespace nc;

namespace
{
bool _parseTopics(ArgParser& parser, bool* allTopicsOut, std::vector<std::string>* topicsOut)
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
} // namespace

DeamonCommand::DeamonCommand() : m_event(false, true)
{
}

DeamonCommand::~DeamonCommand()
{
    m_event.set();

    if (m_writeThread.joinable())
        m_writeThread.join();

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
    if (!_parseTopics(parser, &m_allTopics, &m_topics))
        return false;

    parser.setDefault("limit", "300");
    m_timeLimit = atof(parser.getArg("limit"));

    return true;
}

bool DeamonCommand::checkQueue(ros::Time& time)
{
    if (!m_buffer.empty() && time < m_buffer.back().m_time)
    {
        ROS_WARN("Time has gone backwards, clearing buffer for this topic.");
        m_buffer.clear();
        return false;
    }

    if (!m_buffer.empty())
    {
        float dt = (time - m_buffer.front().m_time).toSec();
        while (dt > m_timeLimit)
        {
            m_buffer.pop_front();
            dt = (time - m_buffer.front().m_time).toSec();
        }
    }

    return true;
}

void DeamonCommand::topicCB(const std::string& topic,
                            const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event)
{
    ros::Time recv_time = ros::Time::now();
    OutgoingMessage msg{topic, recv_time, msg_event.getMessage(), msg_event.getConnectionHeaderPtr()};

    if (msg.m_connectionHeader)
    {
        auto it = msg.m_connectionHeader->find("latching");
        if (it != msg.m_connectionHeader->end() && it->second == "1")
        {
            {
                nc::LockGuard lg(m_bufferMutex);
                m_latchedMsgs[msg.m_topic] = msg;
            }
        }
        else
        {
            if (!checkQueue(msg.m_time))
                return;

            {
                nc::LockGuard lg(m_bufferMutex);
                m_buffer.push_back(msg);
            }
        }
    }
}

void DeamonCommand::subscribeTopic(std::string& topic)
{
    ROS_INFO("Subscribing to %s", topic.c_str());

    ros::SubscribeOptions ops;
    ops.topic = topic;
    ops.queue_size = 100;
    ops.md5sum = ros::message_traits::md5sum<topic_tools::ShapeShifter>();
    ops.datatype = ros::message_traits::datatype<topic_tools::ShapeShifter>();
    ops.helper =
        boost::make_shared<ros::SubscriptionCallbackHelperT<const ros::MessageEvent<topic_tools::ShapeShifter const>&>>(
            [this, topic](const ros::MessageEvent<topic_tools::ShapeShifter const>& ev) { topicCB(topic, ev); });
    m_subscribers.push_back(m_nh.subscribe(ops));
}

void DeamonCommand::pollTopics()
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

bool DeamonCommand::writeTopic(rosbag::Bag& bag, const OutgoingMessage& msg, TriggerRecord::Request& req,
                               TriggerRecord::Response& res)
{
    if (!bag.isOpen())
    {
        try
        {
            bag.open(req.filename, rosbag::bagmode::Write);
            // first write latched topics
            if (!m_latchedMsgs.empty())
            {
                std::map<std::string, OutgoingMessage> latched_msgs;
                {
                    LockGuard lg(m_bufferMutex);
                    latched_msgs = m_latchedMsgs;
                }

                for (auto& lm : latched_msgs)
                {
                    bag.write(lm.first, m_writer.m_startTime, lm.second.m_topicMsg, lm.second.m_connectionHeader);
                }
                std::map<std::string, OutgoingMessage>().swap(latched_msgs);
                malloc_trim(0);
            }
        }
        catch (rosbag::BagException const& err)
        {
            res.success = false;
            res.message = std::string("failed to open bag: ") + err.what();
            return false;
        }
        ROS_INFO("Writing record to %s", req.filename.c_str());
    }

    try
    {
        bag.write(msg.m_topic, msg.m_time, msg.m_topicMsg, msg.m_connectionHeader);
    }
    catch (rosbag::BagException const& err)
    {
        res.success = false;
        res.message = std::string("failed to write bag: ") + err.what();
        return false;
    }

    return true;
}

void DeamonCommand::writeFile()
{
    while (ros::ok())
    {
        m_event.wait();

        if (!ros::ok())
        {
            break;
        }

        rosbag::Bag bag;
        if (m_writer.m_req.compression == 2)
        {
            ROS_INFO("Bag compression type LZ4");
            bag.setCompression(rosbag::compression::LZ4);
        }
        else if (m_writer.m_req.compression == 1)
        {
            ROS_INFO("Bag compression type BZ2");
            bag.setCompression(rosbag::compression::BZ2);
        }
        else
        {
            bag.setCompression(rosbag::compression::Uncompressed);
        }

        std::deque<OutgoingMessage> normal_msgs;
        {
            LockGuard lg(m_bufferMutex);
            normal_msgs = m_buffer;
        }

        for (auto& nm : normal_msgs)
        {
            if (!writeTopic(bag, nm, m_writer.m_req, m_writer.m_res))
                return;
        }
        std::deque<OutgoingMessage>().swap(normal_msgs);
        normal_msgs.shrink_to_fit();

        m_writer.m_readyWrite = false;
    }
}

bool DeamonCommand::triggerRecordCB(TriggerRecord::Request& req, TriggerRecord::Response& res)
{
    if (m_writer.m_readyWrite)
    {
        ROS_WARN("The last time to write file is not over yet!");
        return false;
    }

    res.success = true;
    m_writer.m_startTime = m_buffer.front().m_time;
    m_writer.m_req = req;
    m_writer.m_res = res;
    m_writer.m_readyWrite = true;
    m_event.set();

    return true;
}

int DeamonCommand::run()
{
    if (!m_nh.ok())
        return 1;

    m_triggerServer = m_nh.advertiseService("trigger_record", &DeamonCommand::triggerRecordCB, this);

    m_writeThread = std::thread(&DeamonCommand::writeFile, this);

    if (m_allTopics)
    {
        printf("recoding all topics\n");
        m_pollTopicTimer = m_nh.createTimer(ros::Duration(1.0), [this](const ros::TimerEvent&) { pollTopics(); });
    }
    else
    {
        printf("recoding topics:\n");
        for (auto& topic : m_topics)
        {
            auto res = m_checkTopics.insert(topic);
            if (res.second)
            {
                printf("\t%s\n", topic.c_str());
                subscribeTopic(topic);
            }
        }
    }

    ros::spin();

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
    if (!_parseTopics(parser, &m_allTopics, &m_topics))
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
    ros::ServiceClient client = m_nh.serviceClient<TriggerRecord>("trigger_record");
    if (!client.exists())
    {
        ROS_ERROR("Service %s does not exist. Is record running in this namespace?", "trigger_record");
        return 1;
    }

    TriggerRecordRequest req;
    req.filename = m_filename;

    switch (m_compressType)
    {
    case CompressionType::none:
        req.compression = 0;
        break;
    case CompressionType::bz2:
        req.compression = 1;
        break;
    case CompressionType::lz4:
        req.compression = 2;
        break;
    default:
        break;
    }

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

    if (parser.getArg("help", "h"))
        return cmd;

    if (!cmd->parseArguments(parser))
        return nullptr;

    return cmd;
}
