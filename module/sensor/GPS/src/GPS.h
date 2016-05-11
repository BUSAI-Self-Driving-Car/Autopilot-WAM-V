#ifndef MODULE_SENSOR_GPS_H
#define MODULE_SENSOR_GPS_H

#include <nuclear>

namespace module {
namespace sensor {

    class GPS : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the GPS reactor.
        explicit GPS(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_SENSOR_GPS_H
