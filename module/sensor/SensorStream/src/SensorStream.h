#ifndef MODULE_SENSOR_SENSORSTREAM_H
#define MODULE_SENSOR_SENSORSTREAM_H

#include <nuclear>

namespace module {
namespace sensor {

    class SensorStream : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the SensorStream reactor.
        explicit SensorStream(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_SENSOR_SENSORSTREAM_H
