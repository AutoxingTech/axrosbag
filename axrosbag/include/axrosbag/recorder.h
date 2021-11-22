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
#include <axrosbag_msgs/TriggerRecord.h>
#include <rosgraph_msgs/TopicStatistics.h>
#include <std_srvs/SetBool.h>
#include <string>
#include <topic_tools/shape_shifter.h>
#include <utility>
#include <vector>

namespace axrosbag
{

struct TopicOptions
{
    TopicOptions(ros::Duration duration_limit = INHERIT_DURATION_LIMIT);

    static const ros::Duration NO_DURATION_LIMIT;
    static const ros::Duration INHERIT_DURATION_LIMIT;

    ros::Duration duration_limit_;
};

struct RecorderOptions
{
    RecorderOptions(ros::Duration default_duration_limit = ros::Duration(300));
    bool addTopic(std::string const& topic, ros::Duration duration_limit = TopicOptions::INHERIT_DURATION_LIMIT);

    ros::Duration default_duration_limit_;
    bool all_topics_;
    typedef std::map<std::string, TopicOptions> topics_t;
    topics_t topics_;
    rosbag::compression::CompressionType compression_;
};

struct OutgoingMessage
{
    OutgoingMessage(topic_tools::ShapeShifter::ConstPtr _msg, boost::shared_ptr<ros::M_string> _connection_header,
                    ros::Time _time);

    topic_tools::ShapeShifter::ConstPtr msg;
    boost::shared_ptr<ros::M_string> connection_header;
    ros::Time time;
};

class Recorder;

class MessageQueue
{
    friend Recorder;

private:
    boost::mutex lock;
    TopicOptions options_;

    size_t queue_size_;
    typedef std::deque<OutgoingMessage> queue_t;
    queue_t queue_;

    boost::shared_ptr<ros::Subscriber> sub_;

public:
    explicit MessageQueue(TopicOptions const& options);

    void setSubscriber(boost::shared_ptr<ros::Subscriber> sub);

    typedef std::pair<queue_t::const_iterator, queue_t::const_iterator> range_t;
    range_t rangeFromTimes(ros::Time const& start, ros::Time const& end);

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
    RecorderOptions options_;
    typedef std::map<std::string, boost::shared_ptr<MessageQueue>> buffers_t;
    buffers_t buffers_;
    boost::upgrade_mutex state_lock_;
    bool recording_;
    bool writing_;
    ros::NodeHandle nh_;
    ros::ServiceServer trigger_record_server_;
    ros::ServiceServer enable_server_;
    ros::Timer poll_topic_timer_;

    void fixTopicOptions(TopicOptions& options);
    bool postfixFilename(std::string& file);
    std::string timeAsStr();
    void subscribe(std::string const& topic, boost::shared_ptr<MessageQueue> queue);
    void topicCB(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event,
                 boost::shared_ptr<MessageQueue> queue);
    bool triggerRecordCB(axrosbag_msgs::TriggerRecord::Request& req, axrosbag_msgs::TriggerRecord::Response& res);
    bool enableCB(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    void pause();
    void resume();
    void pollTopics(ros::TimerEvent const& e, axrosbag::RecorderOptions* options);
    bool writeTopic(rosbag::Bag& bag, MessageQueue& msg_queue, std::string const& topic,
                    axrosbag_msgs::TriggerRecord::Request& req, axrosbag_msgs::TriggerRecord::Response& res);
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

    Action action_;
    std::vector<std::string> topics_;
    std::string filename_;
    std::string prefix_;
};

class RecorderClient
{
public:
    RecorderClient();
    int run(RecorderClientOptions const& opts);

private:
    ros::NodeHandle nh_;
};

} // namespace axrosbag

#endif // RECORDER_H_
