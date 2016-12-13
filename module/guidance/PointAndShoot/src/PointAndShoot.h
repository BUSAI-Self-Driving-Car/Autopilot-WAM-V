#ifndef MODULE_GUIDANCE_POINTANDSHOOT_H
#define MODULE_GUIDANCE_POINTANDSHOOT_H

#include <nuclear>
#include <mutex>
#include <atomic>

namespace module {
namespace guidance {

    class PointAndShoot : public NUClear::Reactor {
        std::atomic_bool stop;
        double runTimeSec;
        double throttlePercentage;
        NUClear::clock::time_point start_tp;

    public:
        /// @brief Called by the powerplant to build and setup the PointAndShoot reactor.
        explicit PointAndShoot(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_GUIDANCE_POINTANDSHOOT_H
