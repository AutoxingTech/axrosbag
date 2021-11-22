#include <rosbag/exceptions.h>
#include <axrosbag/recorder.h>
#include <boost/program_options.hpp>
#include <sstream>
#include <string>
#include <vector>

using namespace axrosbag;

namespace po = boost::program_options;

bool parseOptions(po::variables_map& vm, int argc, char** argv)
{
    ros::V_string cleaned_args;
    ros::removeROSArgs(argc, argv, cleaned_args);
    int cleaned_argc = cleaned_args.size();
    char const* cleaned_argv[cleaned_argc];
    for (int i = 0; i < cleaned_argc; ++i)
        cleaned_argv[i] = cleaned_args[i].c_str();

    po::options_description desc("Options");
    // clang-format off
    desc.add_options()
        ("help,h", "produce help message")
        ("trigger-write,t", "Write buffer of selected topcis to a bag file")
        ("pause,p", "Stop buffering new messages until resumed or write is triggered")
        ("resume,r", "Resume buffering new messages, writing over older messages as needed")
        ("all,a", "Record all topics")
        ("bz2,j", "use BZ2 compression")
        ("lz4", "use LZ4 compression")
        ("duration,d", po::value<double>()->default_value(300.0),
        "Maximum difference between newest and oldest buffered message per topic in seconds. Default: 300")
        ("output-prefix,o", po::value<std::string>()->default_value(""),
        "When in trigger write mode, prepend PREFIX to name of writting bag file")
        ("output-filename,O", po::value<std::string>(), "When in trigger write mode, exact name of written bag file")
        ("topic", po::value<std::vector<std::string> >(),
        "Topic to buffer. If triggering write, write only these topics instead of all buffered topics.");
    // clang-format on
    po::positional_options_description p;
    p.add("topic", -1);

    try
    {
        po::store(po::command_line_parser(cleaned_argc, cleaned_argv).options(desc).positional(p).run(), vm);
        po::notify(vm);
    }
    catch (boost::program_options::error const& e)
    {
        std::cout << "axrosbag: " << e.what() << std::endl;
        return false;
    }

    if (vm.count("help"))
    {
        std::cout << "Usage: rosrun axrosbag [options] [topic1 "
                     "topic2 ...]"
                  << std::endl
                  << std::endl
                  << "Buffer recent messages until triggered to write or trigger "
                     "an already running instance."
                  << std::endl
                  << std::endl;
        std::cout << desc << std::endl;
        return false;
    }
    return true;
}

bool parseVariablesMap(RecorderOptions& opts, po::variables_map const& vm)
{
    if (vm.count("topic"))
    {
        std::vector<std::string> topics = vm["topic"].as<std::vector<std::string>>();
        for (const std::string& str : topics)
        {
            opts.addTopic(str);
        }
    }

    opts.default_duration_limit_ = ros::Duration(vm["duration"].as<double>());
    opts.all_topics_ = vm.count("all");
    opts.compression_ = rosbag::compression::LZ4; // or BZ2
    return true;
}

bool parseVariablesMapClient(RecorderClientOptions& opts, po::variables_map const& vm)
{
    if (vm.count("pause"))
        opts.action_ = RecorderClientOptions::PAUSE;
    else if (vm.count("resume"))
        opts.action_ = RecorderClientOptions::RESUME;
    else if (vm.count("trigger-write"))
    {
        opts.action_ = RecorderClientOptions::TRIGGER_WRITE;
        if (vm.count("topic"))
            opts.topics_ = vm["topic"].as<std::vector<std::string>>();
        if (vm.count("output-prefix"))
            opts.prefix_ = vm["output-prefix"].as<std::string>();
        if (vm.count("output-filename"))
            opts.filename_ = vm["output-filename"].as<std::string>();
    }
    return true;
}

void appendParamOptions(ros::NodeHandle& nh, RecorderOptions& opts)
{
    using XmlRpc::XmlRpcValue;
    XmlRpcValue topics;

    // Override program options for default limits if the parameters are set.
    double tmp;
    if (nh.getParam("default_duration_limit", tmp))
        opts.default_duration_limit_ = ros::Duration(tmp);
    nh.param("record_all_topics", opts.all_topics_, false);

    if (!nh.getParam("topics", topics))
    {
        return;
    }
    ROS_ASSERT_MSG(topics.getType() == XmlRpcValue::TypeArray, "topics param must be an array");
}

int main(int argc, char** argv)
{
    po::variables_map vm;
    if (!parseOptions(vm, argc, argv))
        return 1;

    // Parse the command-line options
    if (vm.count("trigger-write") || vm.count("pause") || vm.count("resume"))
    {
        RecorderClientOptions opts;
        if (!parseVariablesMapClient(opts, vm))
            return 1;
        ros::init(argc, argv, "snapshot_client", ros::init_options::AnonymousName);
        RecorderClient client;
        return client.run(opts);
    }

    RecorderOptions opts;
    if (!parseVariablesMap(opts, vm))
        return 1;

    ros::init(argc, argv, "snapshot", ros::init_options::AnonymousName);
    ros::NodeHandle private_nh("~");
    appendParamOptions(private_nh, opts);

    if (opts.topics_.empty() && !opts.all_topics_)
    {
        ROS_FATAL("No topics selected. Exiting.");
        return 1;
    }

    Recorder rec(opts);
    return rec.run();
}
