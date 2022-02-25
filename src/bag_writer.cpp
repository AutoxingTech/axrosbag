#include "bag_writer.h"

bool BagWriter::writeIntoFile(const std::string& filename, CompressionType compressionType, int durationLimit)
{
    try
    {
        m_bag.open(filename, rosbag::bagmode::Write);
    }
    catch (rosbag::BagException const& err)
    {
        m_error = std::string("failed to open bag: ") + err.what();
        return false;
    }

    if (compressionType == CompressionType::lz4)
    {
        ROS_INFO("Bag compression type LZ4");
        m_bag.setCompression(rosbag::compression::LZ4);
    }
    else if (compressionType == CompressionType::bz2)
    {
        ROS_INFO("Bag compression type BZ2");
        m_bag.setCompression(rosbag::compression::BZ2);
    }
    else
    {
        m_bag.setCompression(rosbag::compression::Uncompressed);
    }

    try
    {
        // first write latched topics
        if (!m_latchedMsgs.empty() && !m_messages.empty())
        {
            ros::Time startTime = m_messages.front().recvTime;
            ros::Time endTime = m_messages.back().recvTime;
            if ((endTime - startTime).toSec() > durationLimit)
            {
                startTime = endTime - ros::Duration(durationLimit);
            }

            for (auto& msg : m_latchedMsgs)
            {
                m_bag.write(msg.second.topic, startTime, msg.second.topicMsg, msg.second.connectionHeader);
                if (!ros::ok())
                {
                    m_error = std::string("write cancelled");
                    return false;
                }
            }
        }

        // write topics
        if (!m_messages.empty())
        {
            ros::Time endTime = m_messages.back().recvTime;
            for (auto& msg : m_messages)
            {
                if ((endTime - msg.recvTime).toSec() > durationLimit)
                {
                    continue;
                }

                m_bag.write(msg.topic, msg.recvTime, msg.topicMsg, msg.connectionHeader);
                if (!ros::ok())
                {
                    m_error = std::string("write cancelled");
                    return false;
                }
            }
        }
    }
    catch (rosbag::BagException const& err)
    {
        m_error = std::string("failed to write bag: ") + err.what();
        return false;
    }

    return true;
}
