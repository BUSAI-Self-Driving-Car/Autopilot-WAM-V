#ifndef MODULE_COMMUNICATION_ERRORFORWARDER_H
#define MODULE_COMMUNICATION_ERRORFORWARDER_H

#include <nuclear>

namespace module {
namespace communication {

    class ErrorForwarder : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the ErrorForwarder reactor.
        explicit ErrorForwarder(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_COMMUNICATION_ERRORFORWARDER_H
