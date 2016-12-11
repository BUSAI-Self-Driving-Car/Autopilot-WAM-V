#ifndef MODULE_SENSOR_IMU_H
#define MODULE_SENSOR_IMU_H

#include <nuclear>
#include <utility/io/uart.h>
#include "datatransfer/p2p_connector.hpp"
#include "datatransfer/std_function_callback_handler.hpp"
#include "serialization_policy.hpp"

namespace module {
namespace sensor {

    class IMU : public NUClear::Reactor {
    private:
        using serialization_policy = module::sensor::dto::serialization_policy;
        using p2p_type = datatransfer::p2p_connector<std::mutex, utility::io::uart, serialization_policy, datatransfer::std_function_callback_handler<serialization_policy>>;

        utility::io::uart uart;
        ReactionHandle uart_handle;
        p2p_type p2p;
        bool emitNetwork;

    public:
        /// @brief Called by the powerplant to build and setup the IMU reactor.
        explicit IMU(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_SENSOR_IMU_H
