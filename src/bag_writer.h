#pragma once

#include <deque>
#include <map>
#include <rosbag/bag.h>
#include "common_types.h"

class BagWriter
{
public:
    BagWriter(const std::deque<OutgoingMessage>& buffer,
              const std::map<std::pair<std::string, std::string>, OutgoingMessage>& latchedMsgs)
        : m_messages(buffer), m_latchedMsgs(latchedMsgs)
    {
    }
    ~BagWriter() = default;

    bool writeIntoFile(const std::string& filename, CompressionType compressionType);

    std::string errorMessage() { return m_error; }

private:
    std::string m_error;
    rosbag::Bag m_bag;

    std::deque<OutgoingMessage> m_messages;
    std::map<std::pair<std::string, std::string>, OutgoingMessage> m_latchedMsgs;
};
