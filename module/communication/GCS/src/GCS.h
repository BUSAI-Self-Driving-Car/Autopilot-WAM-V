#ifndef MODULE_COMMUNICATION_GCS_H
#define MODULE_COMMUNICATION_GCS_H

#include <nuclear>
#include <mutex>
#include "message/communication/Status.h"
#include "message/status/Mode.h"


namespace module {
namespace communication {

    class GCS : public NUClear::Reactor {

        message::communication::Status lastStatus;
        uint manual_mode_type;
        message::status::Mode::Type mode;

        void emitMode();
    public:
        /// @brief Called by the powerplant to build and setup the GCS reactor.
        explicit GCS(std::unique_ptr<NUClear::Environment> environment);
    };
}
}

#endif  // MODULE_COMMUNICATION_GCS_H
