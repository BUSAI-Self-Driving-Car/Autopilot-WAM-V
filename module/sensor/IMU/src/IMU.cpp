#include "IMU.h"

#include "extension/Configuration.h"
#include "message/sensor/IMURaw.h"
#include <Eigen/Core>

namespace module {
namespace sensor {

    using extension::Configuration;

    IMU::IMU(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , p2p(uart)
    {

        on<Configuration>("IMU.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file IMU.yaml
            uart.open(config["device"].as<std::string>(), config["baud"].as<unsigned int>());

            p2p.registerMessageHandler<serialization_policy::IMU_MEASUREMENT>([this] (const serialization_policy::data<serialization_policy::IMU_MEASUREMENT>::type& imu)
            {
                using data_type = serialization_policy::data<serialization_policy::IMU_MEASUREMENT>::type;

                auto msg = std::make_unique<message::sensor::IMURaw>();
                msg->accelerometer = Eigen::Map<const Eigen::Vector3f>(imu.accelerometer);
                msg->gyroscope = Eigen::Map<const Eigen::Vector3f>(imu.gyroscope);
                msg->magnetometer = Eigen::Map<const Eigen::Vector3f>(imu.magnetometer);

                emit(msg);
            });

            uart_handle.unbind();
            uart_handle = on<IO,Priority::HIGH>(uart.native_handle(), IO::READ).then("imu read", [this]
            {
                p2p.read();
            });
        });
    }
}
}
