#include <boost/date_time/local_time/local_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/scope_exit.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/xtime.hpp>
#include <memory>
#include <queue>
#include <ros/assert.h>
#include <ros/ros.h>
// #include <axrosbag/recorder.h>
#include <string>
#include <time.h>
#include <topic_tools/shape_shifter.h>
#include <utility>
#include <vector>

namespace axrosbag
{
const ros::Duration TopicOptions::NO_DURATION_LIMIT = ros::Duration(-1);
const ros::Duration TopicOptions::INHERIT_DURATION_LIMIT = ros::Duration(0);

TopicOptions::TopicOptions(ros::Duration duration_limit) : duration_limit_(duration_limit)
{
}

RecorderOptions::RecorderOptions(ros::Duration default_duration_limit)
    : default_duration_limit_(default_duration_limit), topics_(), compression_(rosbag::compression::Uncompressed)
{
}

bool RecorderOptions::addTopic(std::string const& topic, ros::Duration duration)
{
    TopicOptions ops(duration);
    std::pair<topics_t::iterator, bool> ret;
    ret = topics_.insert(topics_t::value_type(topic, ops));
    return ret.second;
}

OutgoingMessage::OutgoingMessage(topic_tools::ShapeShifter::ConstPtr _msg,
                                 boost::shared_ptr<ros::M_string> _connection_header, ros::Time _time)
    : msg(_msg), connection_header(_connection_header), time(_time)
{
}

MessageQueue::MessageQueue(TopicOptions const& options) : options_(options), queue_size_(0)
{
}

void MessageQueue::setSubscriber(boost::shared_ptr<ros::Subscriber> sub)
{
    sub_ = sub;
}

MessageQueue::range_t MessageQueue::rangeFromTimes(ros::Time const& start, ros::Time const& stop)
{
    range_t::first_type begin = queue_.begin();
    range_t::second_type end = queue_.end();

    if (!start.isZero())
    {
        while (begin != end && (*begin).time < start)
            ++begin;
    }
    if (!stop.isZero())
    {
        while (end != begin && (*(end - 1)).time > stop)
            --end;
    }
    return range_t(begin, end);
}

bool MessageQueue::checkQueue(ros::Time const& time)
{
    if (!queue_.empty() && time < queue_.back().time)
    {
        ROS_WARN("Time has gone backwards, clearing buffer for this topic.");
        queue_.clear();
        queue_size_ = 0;
        return false;
    }

    if (options_.duration_limit_ > TopicOptions::NO_DURATION_LIMIT && queue_.size() != 0)
    {
        ros::Duration dt = time - queue_.front().time;
        while (dt > options_.duration_limit_)
        {
            _pop();
            if (queue_.empty())
                break;

            dt = time - queue_.front().time;
        }
    }

    return true;
}

void MessageQueue::_push(OutgoingMessage const& out_msg)
{
    boost::mutex::scoped_try_lock l(lock);
    if (!l.owns_lock())
    {
        ROS_ERROR("Failed to lock. Time %f", out_msg.time.toSec());
        return;
    }

    if (!checkQueue(out_msg.time))
        return;

    queue_.push_back(out_msg);
    queue_size_ += out_msg.msg->size();
}

OutgoingMessage MessageQueue::_pop()
{
    OutgoingMessage drop = queue_.front();
    queue_.pop_front();
    queue_size_ -= drop.msg->size();

    return drop;
}

void MessageQueue::_clear()
{
    boost::mutex::scoped_lock l(lock);
    queue_.clear();
    queue_size_ = 0;
}

Recorder::Recorder(RecorderOptions const& options) : options_(options), recording_(true), writing_(false)
{
}

Recorder::~Recorder()
{
    for (std::pair<const std::string, boost::shared_ptr<MessageQueue>>& buffer : buffers_)
    {
        buffer.second->sub_->shutdown();
    }
}

void Recorder::fixTopicOptions(TopicOptions& options)
{
    if (options.duration_limit_ == TopicOptions::INHERIT_DURATION_LIMIT)
        options.duration_limit_ = options_.default_duration_limit_;
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
        boost::shared_lock<boost::upgrade_mutex> lock(state_lock_);
        if (!recording_)
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
    *sub = nh_.subscribe(ops);
    queue->setSubscriber(sub);
}

bool Recorder::writeTopic(rosbag::Bag& bag, MessageQueue& msg_queue, std::string const& topic,
                          TriggerRecord::Request& req, TriggerRecord::Response& res)
{
    boost::mutex::scoped_lock l(msg_queue.lock);

    MessageQueue::range_t range = msg_queue.rangeFromTimes(req.start_time, req.stop_time);

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
        for (MessageQueue::range_t::first_type msg_it = range.first; msg_it != range.second; ++msg_it)
        {
            OutgoingMessage const& msg = *msg_it;
            bag.write(topic, msg.time, msg.msg, msg.connection_header);
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
        boost::upgrade_lock<boost::upgrade_mutex> read_lock(state_lock_);
        recording_prior = recording_;
        if (writing_)
        {
            res.success = false;
            res.message = "Already writing";
            return true;
        }
        boost::upgrade_to_unique_lock<boost::upgrade_mutex> write_lock(read_lock);
        if (recording_prior)
            pause();
        writing_ = true;
    }

    BOOST_SCOPE_EXIT(&state_lock_, &writing_, recording_prior, this_)
    {
        boost::unique_lock<boost::upgrade_mutex> write_lock(state_lock_);

        writing_ = false;
        if (recording_prior)
            this_->resume();
    }
    BOOST_SCOPE_EXIT_END

    rosbag::Bag bag;
    bag.setCompression(options_.compression_);

    // Write each selected topic's queue to bag file
    if (req.topics.size() && req.topics.at(0).size())
    {
        for (std::string& topic : req.topics)
        {
            // Resolve and clean topic
            try
            {
                topic = ros::names::resolve(nh_.getNamespace(), topic);
            }
            catch (ros::InvalidNameException const& err)
            {
                ROS_WARN("Requested topic %s is invalid, skipping.", topic.c_str());
                continue;
            }

            // Find the message queue for this topic if it exsists
            buffers_t::iterator found = buffers_.find(topic);
            // If topic not found, error and exit
            if (found == buffers_.end())
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
        for (const buffers_t::value_type& pair : buffers_)
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
    ROS_INFO("Buffering paused");
    recording_ = false;
}

void Recorder::resume()
{
    for (const buffers_t::value_type& pair : buffers_)
    {
        pair.second->_clear();
    }

    recording_ = true;

    ROS_INFO("Buffering resumed and old data cleared.");
}

bool Recorder::enableCB(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    boost::upgrade_lock<boost::upgrade_mutex> read_lock(state_lock_);
    if (req.data && writing_)
    {
        res.success = false;
        res.message = "cannot enable recording while writing.";
        return true;
    }

    // Obtain write lock and update state if requested state is different from current
    if (req.data && !recording_)
    {
        boost::upgrade_to_unique_lock<boost::upgrade_mutex> write_lock(read_lock);
        resume();
    }
    else if (!req.data && recording_)
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
                std::pair<buffers_t::iterator, bool> res = buffers_.insert(buffers_t::value_type(topic, queue));
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
    if (!nh_.ok())
        return 0;

    // Create the queue for each topic and set up the subscriber to add to it on new messages
    for (RecorderOptions::topics_t::value_type& pair : options_.topics_)
    {
        std::string topic = ros::names::resolve(nh_.getNamespace(), pair.first);
        fixTopicOptions(pair.second);
        boost::shared_ptr<MessageQueue> queue;
        queue.reset(new MessageQueue(pair.second));
        std::pair<buffers_t::iterator, bool> res = buffers_.insert(buffers_t::value_type(topic, queue));
        ROS_ASSERT_MSG(res.second, "failed to add %s to topics. Perhaps it is a duplicate?", topic.c_str());
        subscribe(topic, queue);
    }

    trigger_record_server_ = nh_.advertiseService("trigger_record", &Recorder::triggerRecordCB, this);
    enable_server_ = nh_.advertiseService("enable_record_", &Recorder::enableCB, this);

    // Start timer to poll ROS master for topics
    if (options_.all_topics_)
        poll_topic_timer_ =
            nh_.createTimer(ros::Duration(1.0), boost::bind(&Recorder::pollTopics, this, _1, &options_));

    ros::spin();

    return 0;
}

RecorderClientOptions::RecorderClientOptions() : action_(RecorderClientOptions::TRIGGER_WRITE)
{
}

RecorderClient::RecorderClient()
{
}

int RecorderClient::run(RecorderClientOptions const& opts)
{
    if (opts.action_ == RecorderClientOptions::TRIGGER_WRITE)
    {
        ros::ServiceClient client = nh_.serviceClient<TriggerRecord>("trigger_record");
        if (!client.exists())
        {
            ROS_ERROR("Service %s does not exist. Is record running in this namespace?", "trigger_record");
            return 1;
        }
        TriggerRecordRequest req;
        req.topics = opts.topics_;

        if (opts.filename_.empty())
        {
            req.filename = opts.prefix_;
            size_t ind = req.filename.rfind(".bag");
            if (ind != std::string::npos && ind == req.filename.size() - 4)
                req.filename.erase(ind);
        }
        else
        {
            req.filename = opts.filename_;
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
    else if (opts.action_ == RecorderClientOptions::PAUSE || opts.action_ == RecorderClientOptions::RESUME)
    {
        ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("enable_record_");
        if (!client.exists())
        {
            ROS_ERROR("Service %s does not exist. Is record running in this namespace?", "enable_record_");
            return 1;
        }
        std_srvs::SetBoolRequest req;
        req.data = (opts.action_ == RecorderClientOptions::RESUME);
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
