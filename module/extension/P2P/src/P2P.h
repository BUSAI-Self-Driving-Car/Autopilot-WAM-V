#ifndef MODULE_EXTENSION_P2P_H
#define MODULE_EXTENSION_P2P_H

#include <nuclear>

namespace module {
namespace extension {

    class P2P : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the P2P reactor.
        explicit P2P(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_EXTENSION_P2P_H
