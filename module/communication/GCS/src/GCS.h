#ifndef MODULE_COMMUNICATION_GCS_H
#define MODULE_COMMUNICATION_GCS_H

#include <nuclear>
#include <mutex>
#include "utility/io/uart.h"
#include <datatransfer/p2p_connector.hpp>
#include <datatransfer/std_function_callback_handler.hpp>
#include "SerializationPolicy.hpp"

namespace module {
namespace communication {

    class GCS : public NUClear::Reactor {

        using module::communication::SerializationPolicy;
        using p2p_t = datatransfer::p2p_connector<std::mutex, utility::io::uart, SerializationPolicy, datatransfer::std_function_callback_handler<SerializationPolicy>>;
    public:
        /// @brief Called by the powerplant to build and setup the GCS reactor.
        explicit GCS(std::unique_ptr<NUClear::Environment> environment);
    private:
        utility::io::uart uart;
        p2p_t p2p;
    };
}
}

#endif  // MODULE_COMMUNICATION_GCS_H
