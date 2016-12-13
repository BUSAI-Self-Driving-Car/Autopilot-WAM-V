#ifndef MODULE_ACTUATOR_LIGHTTOWER_H
#define MODULE_ACTUATOR_LIGHTTOWER_H

#include <nuclear>
#include <utility/io/uart.h>

namespace module {
namespace actuator {

    class LightTower : public NUClear::Reactor {

        static constexpr size_t MAX_RESPONSE_LENGTH = 1024;
        utility::io::uart uart;
        std::string buffer;

        void process();

    public:
        /// @brief Called by the powerplant to build and setup the LightTower reactor.
        explicit LightTower(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_ACTUATOR_LIGHTTOWER_H
