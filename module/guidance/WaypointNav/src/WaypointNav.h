#ifndef MODULE_GUIDANCE_WAYPOINTNAV_H
#define MODULE_GUIDANCE_WAYPOINTNAV_H

#include <nuclear>

namespace module {
namespace guidance {

    class WaypointNav : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the WaypointNav reactor.
        explicit WaypointNav(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_GUIDANCE_WAYPOINTNAV_H
