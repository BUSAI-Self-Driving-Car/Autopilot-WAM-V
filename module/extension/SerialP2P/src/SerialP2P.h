#ifndef MODULE_EXTENSION_SERIALP2P_H
#define MODULE_EXTENSION_SERIALP2P_H

#include <nuclear>

namespace module {
namespace extension {

    class SerialP2P : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the SerialP2P reactor.
        explicit SerialP2P(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_EXTENSION_SERIALP2P_H
