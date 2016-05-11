#ifndef MODULE_SENSOR_IMU_H
#define MODULE_SENSOR_IMU_H

#include <nuclear>

namespace module {
namespace sensor {

    class IMU : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the IMU reactor.
        explicit IMU(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_SENSOR_IMU_H
