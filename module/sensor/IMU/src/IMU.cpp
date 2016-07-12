#include "IMU.h"

#include "extension/Configuration.h"
#include "message/sensor/IMU.h"
#include <Eigen/Core>

namespace module {
namespace sensor {

    using extension::Configuration;

    IMU::IMU(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("IMU.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file IMU.yaml
        });

        on<Every<50, Per<std::chrono::seconds>>>().then("Measurement", [this] {
            //TODO: Replace this with on<IO> and get imu measurement from driver
            auto msg = std::make_unique<message::sensor::IMU>();

            msg->y.resize(9);
            msg->y.setZero();

            emit(msg);
            log("emit IMU Measurement");
        });
    }
}
}
