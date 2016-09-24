#ifndef MODULE_ACTUATOR_TORQEEDO_H
#define MODULE_ACTUATOR_TORQEEDO_H

#include <nuclear>
#include <utility/io/uart.h>
#include "TorqeedoHAL.h"

namespace module {
namespace actuator {

    class Torqeedo : public NUClear::Reactor {

    utility::io::uart uart;
    std::unique_ptr<TorqeedoHAL> hal;
    int i;
    bool started;

    public:
        /// @brief Called by the powerplant to build and setup the Torqeedo reactor.
        explicit Torqeedo(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_ACTUATOR_TORQEEDO_H
