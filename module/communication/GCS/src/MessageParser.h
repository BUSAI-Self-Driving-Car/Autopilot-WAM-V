#ifndef MODULE_COMMUNICATION_MESSAGEPARSER_H
#define MODULE_COMMUNICATION_MESSAGEPARSER_H

#include "utility/io/uart.h"
#include <vector>
#include <map>
#include <functional>

namespace module {
namespace communication {

    class MessageParser
    {

        static const size_t HEADER_SIZE = 8;
        static const size_t CHECKSUM_SIZE = 4;
        static const size_t TYPE_OFFSET = 3;
        static const size_t SIZE_OFFSET = 4;

        enum PARSE_STATE {
            WAIT,
            TYPE,
            SIZE,
            DATA,
            CRC
        };

    public:
        MessageParser(utility::io::uart& uart);
        void read();
        void reset();
        void registerMessageHandler(uint8_t type, std::function<void(const uint8_t *, size_t)> handler);

    private:
        uint8_t MESSAGE_START[3];
        utility::io::uart& uart;
        std::vector<uint8_t> readbuffer;
        PARSE_STATE parseState;
        uint8_t messageType;
        uint32_t messageSize;
        std::vector<uint8_t> messageData;
        std::map<uint8_t, std::function<void(const uint8_t*, size_t)>>  messageHandlers;

    };
}
}

#endif // MODULE_COMMUNICATION_MESSAGEPARSER_H
