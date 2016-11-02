#ifndef MODULE_SENSOR_DTO_IMU_MEASUREMENT_HPP
#define MODULE_SENSOR_DTO_IMU_MEASUREMENT_HPP

#include <stdint.h>

namespace module {
namespace sensor {
namespace dto {

struct imu_measurement
{
    uint64_t timestamp;
    float accelerometer[3];
    float gyroscope[3];
    float magnetometer[3];

    template <typename policy>
    void method(policy& p)
    {
        p % timestamp % accelerometer % gyroscope % magnetometer;
    }
};

}
}
}

#endif // MODULE_SENSOR_DTO_IMU_MEASUREMENT_HPP
