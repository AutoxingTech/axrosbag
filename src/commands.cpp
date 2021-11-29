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

bool DeamonCommand::checkQueue(ros::Time& time)
{
    if (m_buffer.size() > 0 && time < m_buffer.back().m_time)
    {
        ROS_WARN("Time has gone backwards, clearing buffer for this topic.");
        // m_buffer.clear();
        return false;
    }

    if (m_buffer.size() > 0)
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
    std::lock_guard<std::mutex> lock(m_mutex);
    ros::Time recv_time = msg_event.getReceiptTime();
    if (!checkQueue(recv_time))
        return;

    m_buffer.push_back({topic, recv_time, msg_event.getMessage(), msg_event.getConnectionHeaderPtr()});
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
}

bool DeamonCommand::triggerRecordCB(TriggerRecord::Request& req, TriggerRecord::Response& res)
{
    rosbag::Bag bag;
    if (req.compression == 2)
    {
        ROS_INFO("Bag compression type LZ4");
        bag.setCompression(rosbag::compression::LZ4);
    }
    else if (req.compression == 1)
    {
        ROS_INFO("Bag compression type BZ2");
        bag.setCompression(rosbag::compression::BZ2);
    }
    else
    {
        bag.setCompression(rosbag::compression::Uncompressed);
    }

    std::deque<OutgoingMessage> out;
    m_mutex.lock();
    out = m_buffer; // copy buffer
    m_mutex.unlock();

    for (auto& msg : out)
    {
        if (!writeTopic(bag, msg, req, res))
        {
            res.success = false;
            return false;
        }
    }
    out.clear();

    if (!bag.isOpen())
    {
        res.success = false;
        res.message = res.NO_DATA_MESSAGE;
        return false;
    }

    res.success = true;
    return true;
}

int DeamonCommand::run()
{
    if (!m_nh.ok())
        return 0;

    m_triggerServer = m_nh.advertiseService("trigger_record", &DeamonCommand::triggerRecordCB, this);

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

bool DeamonCommand::writeTopic(rosbag::Bag& bag, const OutgoingMessage& msg, TriggerRecord::Request& req,
                               TriggerRecord::Response& res)
{
    if (!bag.isOpen())
    {
        try
        {
            bag.open(req.filename, rosbag::bagmode::Write);
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

    if (!cmd->parseArguments(parser))
        return nullptr;

    return cmd;
}
