#include "MessageParser.h"
#include "stdexcept"
#include <memory>
#include <algorithm>
#include <iterator>
#include <boost/crc.hpp>
#include <iostream>
#include <iomanip>

using namespace module::communication;

MessageParser::MessageParser(utility::io::uart &uart)
    : uart(uart)
    , parseState(PARSE_STATE::WAIT)
    , messageType(0)
{
     MESSAGE_START[0] = 0x17;
     MESSAGE_START[1] = 0xC0;
     MESSAGE_START[2] = 0x42;
}

void MessageParser::reset() {
    parseState = PARSE_STATE::WAIT;
    messageType = 0;
}

void MessageParser::registerMessageHandler(uint32_t type, std::function<void(const std::vector<char> &)> handler)
{
    messageHandlers[type] = handler;
}

void MessageParser::read()
{
    char c;
    auto bytes_read = uart.read(&c,1);

    if (bytes_read > 1) {
        throw std::logic_error("uart read more bytes than requested");
    }
    else if (bytes_read < 1 ) return;

    readbuffer.push_back(c);

    auto len = readbuffer.size();

    // Check if the last 3 bytes signifiy start of a new message
    if (len > 2) {

        if (readbuffer.at(len-3) == MESSAGE_START[0] &&  readbuffer.at(len-2) == MESSAGE_START[1]  &&  readbuffer.at(len-1) == MESSAGE_START[2])
        {
            readbuffer.resize(0);
            parseState = PARSE_STATE::TYPE;
            return;
        }
    }

    if (len > 0) {

        switch (parseState) {

        case PARSE_STATE::WAIT :
            return;

        case PARSE_STATE::TYPE :
            if (len == 4) {
                messageType = (*(reinterpret_cast<const uint32_t*>(readbuffer.data())));
                readbuffer.resize(0);
                parseState = PARSE_STATE::SIZE;
            }
            break;

        case PARSE_STATE::SIZE :
            if (len == 4) {
                messageSize = (*(reinterpret_cast<const uint32_t*>(readbuffer.data())));
                readbuffer.resize(0);
                readbuffer.reserve(messageSize);
                parseState = PARSE_STATE::DATA;
            }
            break;

        case PARSE_STATE::DATA :
            if (len == messageSize) {
                 messageData.resize(0);
                 messageData.reserve(len);
                 messageData.insert(messageData.begin(), readbuffer.begin(), readbuffer.end());
                 readbuffer.resize(0);
                 parseState = PARSE_STATE::CRC;
            }
            break;
         case PARSE_STATE::CRC :
            if (len == 4 &&  messageData.size() == messageSize) {
                boost::crc_32_type crc32;
                crc32.process_bytes(messageData.data(), messageData.size());
                std::reverse(readbuffer.begin(), readbuffer.end());
                auto checksum = (*(reinterpret_cast<const boost::crc_32_type::value_type*>(readbuffer.data())));
                if (crc32.checksum() == checksum) {

                    if (messageHandlers.find(messageType) != messageHandlers.end()) {
                        messageHandlers[messageType](messageData);
                    }
                }               

                readbuffer.resize(0);
                parseState = PARSE_STATE::WAIT;
            }
            break;

        default:
            throw std::runtime_error("Unknown PARSE STATE");
        }
    }
}
