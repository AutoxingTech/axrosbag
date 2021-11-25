#ifndef RECORDER_H_
#define RECORDER_H_

#include <boost/atomic.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <deque>
#include <map>
#include <ros/ros.h>
#include <ros/time.h>
#include <rosbag/bag.h>
#include <rosbag/macros.h>
#include <std_srvs/SetBool.h>
#include <string>
#include <topic_tools/shape_shifter.h>
#include <utility>
#include <vector>
#include <axrosbag/TriggerRecord.h>

namespace axrosbag
{

struct TopicOptions
{
    TopicOptions(ros::Duration duration_limit = INHERIT_DURATION_LIMIT);

    static const ros::Duration NO_DURATION_LIMIT;
    static const ros::Duration INHERIT_DURATION_LIMIT;
    ros::Duration m_duration_limit;
};

struct RecorderOptions
{
    RecorderOptions(ros::Duration default_duration_limit = ros::Duration(300));
    bool addTopic(std::string const& topic, ros::Duration duration_limit = TopicOptions::INHERIT_DURATION_LIMIT);

    ros::Duration m_duration_limit;
    bool m_all_topics;
    typedef std::map<std::string, TopicOptions> TopicsType;
    TopicsType m_topics;
    rosbag::compression::CompressionType m_compression;
};

struct OutgoingMessage
{
    OutgoingMessage(topic_tools::ShapeShifter::ConstPtr msg, boost::shared_ptr<ros::M_string> connection_header,
                    ros::Time time);

    topic_tools::ShapeShifter::ConstPtr m_msg;
    boost::shared_ptr<ros::M_string> m_connection_header;
    ros::Time m_time;
};

class Recorder;

class MessageQueue
{
    friend Recorder;

private:
    boost::mutex lock;
    TopicOptions m_options;

    size_t m_queue_size;
    typedef std::deque<OutgoingMessage> QueueType;
    QueueType m_queue;

    boost::shared_ptr<ros::Subscriber> m_sub;

public:
    explicit MessageQueue(TopicOptions const& options);
    void setSubscriber(boost::shared_ptr<ros::Subscriber> sub);

    typedef std::pair<QueueType::const_iterator, QueueType::const_iterator> RangeType;
    RangeType rangeFromTimes(ros::Time const& start, ros::Time const& end);

    bool checkQueue(ros::Time const& time);
    void _push(OutgoingMessage const& out_msg);
    OutgoingMessage _pop();
    void _clear();
};

class Recorder
{
public:
    explicit Recorder(RecorderOptions const& options);
    ~Recorder();

    int run();

private:
    RecorderOptions m_options;
    typedef std::map<std::string, boost::shared_ptr<MessageQueue>> BuffersType;
    BuffersType m_buffers;
    boost::upgrade_mutex m_state_lock;
    bool m_recording;
    bool m_writing;
    ros::NodeHandle m_nh;
    ros::ServiceServer m_trigger_record_server;
    ros::ServiceServer m_enable_server;
    ros::Timer m_poll_topic_timer;

    void fixTopicOptions(TopicOptions& options);
    bool postfixFilename(std::string& file);
    std::string timeAsStr();
    void subscribe(std::string const& topic, boost::shared_ptr<MessageQueue> queue);
    void topicCB(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event,
                 boost::shared_ptr<MessageQueue> queue);
    bool triggerRecordCB(TriggerRecord::Request& req, TriggerRecord::Response& res);
    bool enableCB(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    void pause();
    void resume();
    void pollTopics(ros::TimerEvent const& e, axrosbag::RecorderOptions* options);
    bool writeTopic(rosbag::Bag& bag, MessageQueue& msg_queue, std::string const& topic, TriggerRecord::Request& req,
                    TriggerRecord::Response& res);
};

struct RecorderClientOptions
{
    RecorderClientOptions();

    enum Action
    {
        TRIGGER_WRITE,
        PAUSE,
        RESUME
    };

    Action m_action;
    std::vector<std::string> m_topics;
    std::string m_filename;
    std::string m_prefix;
};

class RecorderClient
{
public:
    RecorderClient();
    int run(RecorderClientOptions const& opts);

private:
    ros::NodeHandle m_nh;
};

} // namespace axrosbag

#endif // RECORDER_H_
