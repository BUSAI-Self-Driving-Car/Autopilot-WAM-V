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
        void registerMessageHandler(uint32_t type, std::function<void(const std::vector<char>&)> handler);

    private:
        char MESSAGE_START[3];
        utility::io::uart& uart;
        std::vector<char> readbuffer;
        PARSE_STATE parseState;
        uint32_t messageType;
        uint32_t messageSize;
        std::vector<char> messageData;
        std::map<uint32_t, std::function<void(const std::vector<char>&)>>  messageHandlers;

    };
}
}

#endif // MODULE_COMMUNICATION_MESSAGEPARSER_H
