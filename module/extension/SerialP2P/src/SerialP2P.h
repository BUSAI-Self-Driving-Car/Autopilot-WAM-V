#ifndef MODULE_EXTENSION_SERIALP2P_H
#define MODULE_EXTENSION_SERIALP2P_H

#include <nuclear>
#include "utility/io/uart.h"

namespace module {
namespace extension {

    class SerialP2P : public NUClear::Reactor {

        enum PARSE_STATE {
            WAIT,
            TYPE,
            SIZE,
            DATA,
            CRC
        };

    public:
        /// @brief Called by the powerplant to build and setup the SerialP2P reactor.
        explicit SerialP2P(std::unique_ptr<NUClear::Environment> environment);

    private:
        std::multimap<uint32_t, std::shared_ptr<NUClear::threading::Reaction>> handlers;

        char MESSAGE_START[3];
        utility::io::uart uart;
        std::vector<char> readbuffer;
        PARSE_STATE parseState;
        uint32_t messageType;
        uint32_t messageSize;
        std::vector<char> messageData;
    };
}
}

#endif  // MODULE_EXTENSION_SERIALP2P_H
