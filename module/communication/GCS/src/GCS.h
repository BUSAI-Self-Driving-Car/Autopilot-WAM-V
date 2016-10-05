#ifndef MODULE_COMMUNICATION_GCS_H
#define MODULE_COMMUNICATION_GCS_H

#include <nuclear>
#include <mutex>
#include "utility/io/uart.h"
#include "MessageParser.h"

namespace module {
namespace communication {

    class GCS : public NUClear::Reactor {

       enum MSG_TYPES {
           STATUS=                   0,
           COMMAND=                  1,
           GET_PARAMETERS=           2,
           GET_PARAMETERS_RESPONSE=  3,
           GET_SETTINGS=             4,
           GET_SETTINGS_RESPONSE=    5,
           GET_MISSION=              6,
           GET_MISSION_RESPONSE=     7,
           GET_MISSIONS=             8,
           GET_MISSIONS_RESPONSE=    9,
           SET_PARAMETERS=          10,
           SET_PARAMETERS_ACK=      11,
           SET_SETTINGS=            12,
           SET_SETTINGS_ACK=        13,
           SET_MISSION=             14,
           SET_MISSION_ACK=         15,
           SET_MISSIONS=            16,
           SET_MISSIONS_ACK=        17,
           ATTENTION=               18,
           CONTROLLER_COMMAND=      19,
       };

    public:
        /// @brief Called by the powerplant to build and setup the GCS reactor.
        explicit GCS(std::unique_ptr<NUClear::Environment> environment);
    private:
        utility::io::uart uart;
        MessageParser messageParser;

        void onControllerCommand(const std::vector<uint8_t>& buffer);
    };
}
}

#endif  // MODULE_COMMUNICATION_GCS_H
