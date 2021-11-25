#include <boost/date_time/local_time/local_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/scope_exit.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/xtime.hpp>
#include <memory>
#include <queue>
#include <ros/assert.h>
#include <ros/ros.h>
#include <axrosbag/recorder.h>
#include <string>
#include <time.h>
#include <topic_tools/shape_shifter.h>
#include <utility>
#include <vector>

namespace axrosbag
{
const ros::Duration TopicOptions::NO_DURATION_LIMIT = ros::Duration(-1);
const ros::Duration TopicOptions::INHERIT_DURATION_LIMIT = ros::Duration(0);

TopicOptions::TopicOptions(ros::Duration duration_limit) : m_duration_limit(duration_limit)
{
}

RecorderOptions::RecorderOptions(ros::Duration default_duration_limit)
    : m_duration_limit(default_duration_limit), m_topics(), m_compression(rosbag::compression::Uncompressed)
{
}

bool RecorderOptions::addTopic(std::string const& topic, ros::Duration duration)
{
    TopicOptions ops(duration);
    std::pair<TopicsType::iterator, bool> ret;
    ret = m_topics.insert(TopicsType::value_type(topic, ops));
    return ret.second;
}

OutgoingMessage::OutgoingMessage(topic_tools::ShapeShifter::ConstPtr msg,
                                 boost::shared_ptr<ros::M_string> connection_header, ros::Time time)
    : m_msg(msg), m_connection_header(connection_header), m_time(time)
{
}

MessageQueue::MessageQueue(TopicOptions const& options) : m_options(options), m_queue_size(0)
{
}

void MessageQueue::setSubscriber(boost::shared_ptr<ros::Subscriber> sub)
{
    m_sub = sub;
}

MessageQueue::RangeType MessageQueue::rangeFromTimes(ros::Time const& start, ros::Time const& stop)
{
    RangeType::first_type begin = m_queue.begin();
    RangeType::second_type end = m_queue.end();

    if (!start.isZero())
    {
        while (begin != end && (*begin).m_time < start)
            ++begin;
    }
    if (!stop.isZero())
    {
        while (end != begin && (*(end - 1)).m_time > stop)
            --end;
    }
    return RangeType(begin, end);
}

bool MessageQueue::checkQueue(ros::Time const& time)
{
    if (!m_queue.empty() && time < m_queue.back().m_time)
    {
        ROS_WARN("Time has gone backwards, clearing buffer for this topic.");
        m_queue.clear();
        m_queue_size = 0;
        return false;
    }

    if (m_options.m_duration_limit > TopicOptions::NO_DURATION_LIMIT && m_queue.size() != 0)
    {
        ros::Duration dt = time - m_queue.front().m_time;
        while (dt > m_options.m_duration_limit)
        {
            _pop();
            if (m_queue.empty())
                break;

            dt = time - m_queue.front().m_time;
        }
    }

    return true;
}

void MessageQueue::_push(OutgoingMessage const& out_msg)
{
    boost::mutex::scoped_try_lock l(lock);
    if (!l.owns_lock())
    {
        ROS_ERROR("Failed to lock. Time %f", out_msg.m_time.toSec());
        return;
    }

    if (!checkQueue(out_msg.m_time))
        return;

    m_queue.push_back(out_msg);
    m_queue_size += out_msg.m_msg->size();
}

OutgoingMessage MessageQueue::_pop()
{
    OutgoingMessage drop = m_queue.front();
    m_queue.pop_front();
    m_queue_size -= drop.m_msg->size();

    return drop;
}

void MessageQueue::_clear()
{
    boost::mutex::scoped_lock l(lock);
    m_queue.clear();
    m_queue_size = 0;
}

Recorder::Recorder(RecorderOptions const& options) : m_options(options), m_recording(true), m_writing(false)
{
}

Recorder::~Recorder()
{
    for (std::pair<const std::string, boost::shared_ptr<MessageQueue>>& buffer : m_buffers)
    {
        buffer.second->m_sub->shutdown();
    }
}

void Recorder::fixTopicOptions(TopicOptions& options)
{
    if (options.m_duration_limit == TopicOptions::INHERIT_DURATION_LIMIT)
        options.m_duration_limit = m_options.m_duration_limit;
}

bool Recorder::postfixFilename(std::string& file)
{
    size_t ind = file.rfind(".bag");
    // If requested ends in .bag, this is literal name do not append date
    if (ind != std::string::npos && ind == file.size() - 4)
    {
        return true;
    }
    // Otherwise treat as prefix and append datetime and extension
    file += timeAsStr() + ".bag";
    return true;
}

std::string Recorder::timeAsStr()
{
    std::stringstream msg;
    const boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
    boost::posix_time::time_facet* const f = new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S");
    msg.imbue(std::locale(msg.getloc(), f));
    msg << now;
    return msg.str();
}

void Recorder::topicCB(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event,
                       boost::shared_ptr<MessageQueue> queue)
{
    {
        boost::shared_lock<boost::upgrade_mutex> lock(m_state_lock);
        if (!m_recording)
        {
            return;
        }
    }

    OutgoingMessage out(msg_event.getMessage(), msg_event.getConnectionHeaderPtr(), msg_event.getReceiptTime());
    queue->_push(out);
}

void Recorder::subscribe(std::string const& topic, boost::shared_ptr<MessageQueue> queue)
{
    ROS_INFO("Subscribing to %s", topic.c_str());

    boost::shared_ptr<ros::Subscriber> sub(boost::make_shared<ros::Subscriber>());
    ros::SubscribeOptions ops;
    ops.topic = topic;
    ops.queue_size = 100;
    ops.md5sum = ros::message_traits::md5sum<topic_tools::ShapeShifter>();
    ops.datatype = ros::message_traits::datatype<topic_tools::ShapeShifter>();
    ops.helper =
        boost::make_shared<ros::SubscriptionCallbackHelperT<const ros::MessageEvent<topic_tools::ShapeShifter const>&>>(
            boost::bind(&Recorder::topicCB, this, _1, queue));
    *sub = m_nh.subscribe(ops);
    queue->setSubscriber(sub);
}

bool Recorder::writeTopic(rosbag::Bag& bag, MessageQueue& msg_queue, std::string const& topic,
                          TriggerRecord::Request& req, TriggerRecord::Response& res)
{
    boost::mutex::scoped_lock l(msg_queue.lock);

    MessageQueue::RangeType range = msg_queue.rangeFromTimes(req.start_time, req.stop_time);

    if (!bag.isOpen() && range.second > range.first)
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
        for (MessageQueue::RangeType::first_type msg_it = range.first; msg_it != range.second; ++msg_it)
        {
            OutgoingMessage const& msg = *msg_it;
            bag.write(topic, msg.m_time, msg.m_msg, msg.m_connection_header);
        }
    }
    catch (rosbag::BagException const& err)
    {
        res.success = false;
        res.message = std::string("failed to write bag: ") + err.what();
    }

    return true;
}

bool Recorder::triggerRecordCB(TriggerRecord::Request& req, TriggerRecord::Response& res)
{
    if (!postfixFilename(req.filename))
    {
        res.success = false;
        res.message = "invalid";
        return true;
    }

    bool recording_prior;
    {
        boost::upgrade_lock<boost::upgrade_mutex> read_lock(m_state_lock);
        recording_prior = m_recording;
        if (m_writing)
        {
            res.success = false;
            res.message = "Already writing";
            return true;
        }
        boost::upgrade_to_unique_lock<boost::upgrade_mutex> write_lock(read_lock);
        if (recording_prior)
            pause();
        m_writing = true;
    }

    BOOST_SCOPE_EXIT(&m_state_lock, &m_writing, recording_prior, this_)
    {
        boost::unique_lock<boost::upgrade_mutex> write_lock(m_state_lock);

        m_writing = false;
        if (recording_prior)
            this_->resume();
    }
    BOOST_SCOPE_EXIT_END

    rosbag::Bag bag;
    bag.setCompression(m_options.m_compression);

    // Write each selected topic's queue to bag file
    if (req.topics.size() && req.topics.at(0).size())
    {
        for (std::string& topic : req.topics)
        {
            // Resolve and clean topic
            try
            {
                topic = ros::names::resolve(m_nh.getNamespace(), topic);
            }
            catch (ros::InvalidNameException const& err)
            {
                ROS_WARN("Requested topic %s is invalid, skipping.", topic.c_str());
                continue;
            }

            // Find the message queue for this topic if it exsists
            BuffersType::iterator found = m_buffers.find(topic);
            // If topic not found, error and exit
            if (found == m_buffers.end())
            {
                ROS_WARN("Requested topic %s is not subscribed, skipping.", topic.c_str());
                continue;
            }
            MessageQueue& msg_queue = *(*found).second;
            if (!writeTopic(bag, msg_queue, topic, req, res))
                return true;
        }
    }
    // If topic list empty, record all buffered topics
    else
    {
        for (const BuffersType::value_type& pair : m_buffers)
        {
            MessageQueue& msg_queue = *(pair.second);
            std::string const& topic = pair.first;
            if (!writeTopic(bag, msg_queue, topic, req, res))
                return true;
        }
    }

    if (!bag.isOpen())
    {
        res.success = false;
        res.message = res.NO_DATA_MESSAGE;
        return true;
    }

    res.success = true;
    return true;
}

void Recorder::pause()
{
    m_recording = false;
}

void Recorder::resume()
{
    m_recording = true;
}

bool Recorder::enableCB(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    boost::upgrade_lock<boost::upgrade_mutex> read_lock(m_state_lock);
    if (req.data && m_writing)
    {
        res.success = false;
        res.message = "cannot enable recording while writing.";
        return true;
    }

    // Obtain write lock and update state if requested state is different from current
    if (req.data && !m_recording)
    {
        boost::upgrade_to_unique_lock<boost::upgrade_mutex> write_lock(read_lock);
        resume();
    }
    else if (!req.data && m_recording)
    {
        boost::upgrade_to_unique_lock<boost::upgrade_mutex> write_lock(read_lock);
        pause();
    }
    res.success = true;
    return true;
}

void Recorder::pollTopics(ros::TimerEvent const& e, axrosbag::RecorderOptions* options)
{
    (void)e;
    ros::master::V_TopicInfo topics;
    if (ros::master::getTopics(topics))
    {
        for (ros::master::TopicInfo const& t : topics)
        {
            std::string topic = t.name;
            if (options->addTopic(topic))
            {
                TopicOptions topic_options;
                fixTopicOptions(topic_options);
                boost::shared_ptr<MessageQueue> queue;
                queue.reset(new MessageQueue(topic_options));
                std::pair<BuffersType::iterator, bool> res = m_buffers.insert(BuffersType::value_type(topic, queue));
                ROS_ASSERT_MSG(res.second, "failed to add %s to topics. Perhaps it is a duplicate?", topic.c_str());
                subscribe(topic, queue);
            }
        }
    }
    else
    {
        ROS_WARN_THROTTLE(5, "Failed to get topics from the ROS master");
    }
}

int Recorder::run()
{
    if (!m_nh.ok())
        return 0;

    // Create the queue for each topic and set up the subscriber to add to it on new messages
    for (RecorderOptions::TopicsType::value_type& pair : m_options.m_topics)
    {
        std::string topic = ros::names::resolve(m_nh.getNamespace(), pair.first);
        fixTopicOptions(pair.second);
        boost::shared_ptr<MessageQueue> queue;
        queue.reset(new MessageQueue(pair.second));
        std::pair<BuffersType::iterator, bool> res = m_buffers.insert(BuffersType::value_type(topic, queue));
        ROS_ASSERT_MSG(res.second, "failed to add %s to topics. Perhaps it is a duplicate?", topic.c_str());
        subscribe(topic, queue);
    }

    m_trigger_record_server = m_nh.advertiseService("trigger_record", &Recorder::triggerRecordCB, this);
    m_enable_server = m_nh.advertiseService("enable_record", &Recorder::enableCB, this);

    // Start timer to poll ROS master for topics
    if (m_options.m_all_topics)
        m_poll_topic_timer =
            m_nh.createTimer(ros::Duration(1.0), boost::bind(&Recorder::pollTopics, this, _1, &m_options));

    ros::spin();

    return 0;
}

RecorderClientOptions::RecorderClientOptions() : m_action(RecorderClientOptions::TRIGGER_WRITE)
{
}

RecorderClient::RecorderClient()
{
}

int RecorderClient::run(RecorderClientOptions const& opts)
{
    if (opts.m_action == RecorderClientOptions::TRIGGER_WRITE)
    {
        ros::ServiceClient client = m_nh.serviceClient<TriggerRecord>("trigger_record");
        if (!client.exists())
        {
            ROS_ERROR("Service %s does not exist. Is record running in this namespace?", "trigger_record");
            return 1;
        }
        TriggerRecordRequest req;
        req.topics = opts.m_topics;

        if (opts.m_filename.empty())
        {
            req.filename = opts.m_prefix;
            size_t ind = req.filename.rfind(".bag");
            if (ind != std::string::npos && ind == req.filename.size() - 4)
                req.filename.erase(ind);
        }
        else
        {
            req.filename = opts.m_filename;
            size_t ind = req.filename.rfind(".bag");
            if (ind == std::string::npos || ind != req.filename.size() - 4)
                req.filename += ".bag";
        }

        if (req.filename.empty())
            req.filename = "./";
        boost::filesystem::path p(boost::filesystem::system_complete(req.filename));
        req.filename = p.string();

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
        return 0;
    }
    else if (opts.m_action == RecorderClientOptions::PAUSE || opts.m_action == RecorderClientOptions::RESUME)
    {
        ros::ServiceClient client = m_nh.serviceClient<std_srvs::SetBool>("enable_record");
        if (!client.exists())
        {
            ROS_ERROR("Service %s does not exist. Is record running in this namespace?", "enable_record");
            return 1;
        }
        std_srvs::SetBoolRequest req;
        req.data = (opts.m_action == RecorderClientOptions::RESUME);
        std_srvs::SetBoolResponse res;
        if (!client.call(req, res))
        {
            ROS_ERROR("Failed to call service.");
            return 1;
        }
        if (!res.success)
        {
            ROS_ERROR("%s", res.message.c_str());
            return 1;
        }
        return 0;
    }
    else
    {
        ROS_ASSERT_MSG(false, "Invalid value of enum.");
        return 1;
    }
}

} // namespace axrosbag
