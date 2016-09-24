#include "MessageParser.h"
#include "stdexcept"
#include <memory>
#include <algorithm>
#include <iterator>
#include <boost/crc.hpp>

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

void MessageParser::registerMessageHandler(uint8_t type, std::function<void(const uint8_t*, uint32_t)> handler)
{
    messageHandlers[type] = handler;
}

void MessageParser::read()
{
    if (parseState == PARSE_STATE::DATA && messageSize > readbuffer.size()) {
        uint32_t nBytes =  messageSize - readbuffer.size();
        uart.read(readbuffer.data(), nBytes);
    }
    else {
        uint8_t c;
        auto bytes_read = uart.read(&c,1);

        if (bytes_read > 1) {
            throw std::logic_error("uart read more bytes than requested");
        }
        if (bytes_read == 0) return;

        readbuffer.push_back(c);
    }

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
            if (len == 1) {
                messageType = readbuffer.at(0);
                readbuffer.resize(0);
                parseState = PARSE_STATE::SIZE;
            }
            else {
                throw std::logic_error("Expected the read buffer size to be 1 for message type");
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
                 std::copy(readbuffer.begin(),readbuffer.end(),back_inserter(messageData));
                 readbuffer.resize(0);
                 parseState = PARSE_STATE::CRC;
            }
            break;
         case PARSE_STATE::CRC :
            if (len == 4) {
                boost::crc_32_type crc32;
                crc32.process_bytes(messageData.data(),messageData.size());

                auto checksum = (*(reinterpret_cast<const boost::crc_32_type::value_type*>(readbuffer.data())));
                if (crc32.checksum() == checksum) {

                    if (messageHandlers.find(messageType) != messageHandlers.end()) {
                        messageHandlers[messageType](messageData.data(), messageSize);
                    }
                }
                readbuffer.resize(0);
                parseState = PARSE_STATE::WAIT;
            }
            break;

        default:
            throw std::runtime_error("Unkown PARSE STATE");
        }
    }
}
