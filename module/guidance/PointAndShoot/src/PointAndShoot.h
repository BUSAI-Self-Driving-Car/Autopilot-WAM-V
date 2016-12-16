#ifndef MODULE_GUIDANCE_POINTANDSHOOT_H
#define MODULE_GUIDANCE_POINTANDSHOOT_H

#include <nuclear>
#include <mutex>
#include <atomic>
#include "message/navigation/StateEstimate.h"

namespace module {
namespace guidance {

    class PointAndShoot : public NUClear::Reactor {
        std::atomic_bool stop;
        double runTimeSec;
        double throttlePercentage;
        double forwardForce;
        double heading;
        NUClear::clock::time_point start_tp;

        enum RUN_MODE {
            THROTTLE=0,
            HEADING=1
        };

        RUN_MODE runMode;
        void runthrottleMode(double throttle);
        void runHeadingMode(double force, const message::navigation::StateEstimate &state);

    public:
        /// @brief Called by the powerplant to build and setup the PointAndShoot reactor.
        explicit PointAndShoot(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_GUIDANCE_POINTANDSHOOT_H
