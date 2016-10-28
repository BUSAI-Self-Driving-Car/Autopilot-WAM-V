#include "DataLogging.h"
#include <iomanip>

#include "extension/Configuration.h"
#include "message/communication/GamePad.h"
#include "message/sensor/GPSRaw.h"

namespace module {
namespace support {
namespace logging {

    using extension::Configuration;
    using message::communication::GamePad;
    using message::sensor::GPSRaw;

    DataLogging::DataLogging(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration, Sync<DataLog>>("DataLogging.yaml").then([this] (const Configuration& config) {
            std::string output_dir = config["directory"].as<std::string>();

            // Make the time into a folder pattern
            std::time_t now = time(0);
            std::tm systemTime = *localtime(&now);
            std::stringstream logfile;
            logfile << output_dir
                    << "/"
                    << std::put_time(&systemTime, "%Y%m%dT%H_%M_%S")
                    << ".nbs";

            // Close if required then open that file
            if (output_file.is_open()) {
                output_file.close();
            }

            log(logfile.str());

            output_file = std::ofstream(logfile.str());
        });

        on<Trigger<DataLog>, Sync<DataLog>>().then([this] (const DataLog& data) {

            // FILE FORMAT
            // TYPE       DATA
            // char[3]    RADIATION SYMBOL { 0xE2, 0x98, 0xA2 }
            // uint32_t   SIZE OF NEXT PACKET
            // uint64_t   TIMESTAMP DATA WAS EMITTED
            // uint128_t  DATA TYPE HASH

            // If the file isn't open skip
            if (!output_file.is_open()) {
                return;
            }

            using namespace std::chrono;

            uint64_t timestamp_us = duration_cast<duration<uint64_t, std::micro>>(data.timestamp.time_since_epoch()).count();

            // The size of our output timestamp hash and data
            uint32_t size = data.data.size() + sizeof(data.hash) + sizeof(timestamp_us);

            // Write radiation symbol
            output_file.put(0xE2);
            output_file.put(0x98);
            output_file.put(0xA2);

            // Write the size of the packet
            output_file.write(reinterpret_cast<const char*>(&size), sizeof(size));

            // Write the timestamp
            output_file.write(reinterpret_cast<const char*>(&timestamp_us), sizeof(timestamp_us));

            // Write the hash
            output_file.write(reinterpret_cast<const char*>(&data.hash), sizeof(data.hash));

            // Write the acutal packet data
            output_file.write(data.data.data(), data.data.size());
            output_file.flush();
        });

        // Our actual datalogging
        on<Trigger<GamePad>>().then([this] (const GamePad& d) {
            emit(log_encode(d));
        });

        on<Trigger<GPSRaw>>().then([this] (const GPSRaw& d) {
            emit(log_encode(d));
        });
    }
}
}
}
