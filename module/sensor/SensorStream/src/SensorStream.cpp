#include "SensorStream.h"

#include "extension/Configuration.h"
#include "message/sensor/GPSRaw.h"

namespace module {
namespace sensor {

    using extension::Configuration;
    using message::sensor::GPSRaw;

    #pragma pack(push, 1)

    struct SensorPacket {
        // Header
        char header[2];
        uint8_t version;
        uint8_t size;

        // Body
        float accelerometer[3];
        float gyroscope[3];
        double teslameter[3];
        double magnetic_heading;
        double true_heading;

        double latitude;
        double longitude;
        double altitutde;

        bool proximity_sensor;
        bool p1_touched;
        int32_t p1_touched_pos[2];

        bool p2_touched;
        int32_t p2_touched_pos[2];
    };

    #pragma pack(pop)

    SensorStream::SensorStream(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("SensorStream.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file SensorStream.yaml

            int port;
            std::tie(std::ignore, port, std::ignore) = on<UDP>(config["port"].as<uint>()).then([this] (const UDP::Packet& data) {

                const SensorPacket& packet = *reinterpret_cast<const SensorPacket*>(data.payload.data());

                auto msg = std::make_unique<GPSRaw>();

                // We always say we have a GPS fix
                msg->fix_type = GPSRaw::FixType::GPS_FIX;
                msg->lla = Eigen::Vector3d(packet.latitude, packet.longitude, packet.altitutde);


                // NaN for everything we don't know
                msg->velocity = Eigen::Vector2d(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
                msg->pdop = std::numeric_limits<double>::quiet_NaN();
                msg->hdop = std::numeric_limits<double>::quiet_NaN();
                msg->vdop = std::numeric_limits<double>::quiet_NaN();

                emit(msg);

                // log("Version:", packet.version);
                // log("Size:", packet.size);


                // log("Accelerometer:", packet.accelerometer[0], packet.accelerometer[1], packet.accelerometer[2]);
                // log("Gyroscope:", packet.gyroscope[0], packet.gyroscope[1], packet.gyroscope[2]);
                // log("Teslameter:", packet.teslameter[0], packet.teslameter[1], packet.teslameter[2]);

                // log("Magnetic Heading:", packet.magnetic_heading);
                // log("True Heading:", packet.true_heading);

                // log("GPS:", packet.latitude, packet.longitude, packet.altitutde);
            });

            log("Listening for SensorStream on port:", port);

        });
    }
}
}
