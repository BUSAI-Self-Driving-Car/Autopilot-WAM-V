#include "DataLogging.h"
#include <iomanip>

#include "extension/Configuration.h"
#include "utility/file/fileutil.h"
#include "message/communication/GamePad.h"
#include "message/propulsion/TorqeedoStatus.h"
#include "message/sensor/GPSRaw.h"
#include "message/sensor/IMURaw.h"
#include "message/navigation/StateEstimate.h"
#include "message/vision/Image.h"
#include "message/vision/CompressedImage.h"

namespace module {
namespace support {
namespace logging {

    using extension::Configuration;
    using message::communication::GamePad;
    using message::sensor::GPSRaw;
    using message::sensor::IMURaw;
    using message::propulsion::TorqeedoStatus;
    using message::navigation::StateEstimate;
    using message::vision::Image;
    using message::vision::CompressedImage;

    DataLogging::DataLogging(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

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
        handles["message.communication.GamePad"] = on<Trigger<GamePad>>().then([this] (const GamePad& d) {
            emit(log_encode(d));
        }).disable();

        handles["message.sensor.GPSRaw"] = on<Trigger<GPSRaw>>().then([this] (const GPSRaw& d) {
            emit(log_encode(d));
        }).disable();

        handles["message.sensor.IMURaw"] = on<Trigger<IMURaw>>().then([this] (const IMURaw& d) {
            emit(log_encode(d));
        }).disable();

        handles["message.navigation.StateEstimate"] = on<Trigger<StateEstimate>>().then([this] (const StateEstimate& d) {
            emit(log_encode(d));
        }).disable();

        handles["message.vision.Image"] = on<Trigger<Image>>().then([this] (const Image& d) {
            emit(log_encode(d));
        }).disable();

        handles["message.vision.CompressedImage"] = on<Trigger<CompressedImage>>().then([this] (const CompressedImage& d) {
            emit(log_encode(d));
        }).disable();

        handles["message.propulsion.TorqueedoStatus"] = on<Trigger<TorqeedoStatus>>().then([this] (const TorqeedoStatus& d) {
           emit(log_encode(d));
        }).disable();

        on<Configuration, Trigger<NUClear::message::CommandLineArguments>, Sync<DataLog>>("DataLogging.yaml").then([this] (const Configuration& config, const NUClear::message::CommandLineArguments& argv) {
            std::string output_dir = config["directory"].as<std::string>();

            // Make the time into a folder pattern
            std::time_t now = time(0);
            std::tm systemTime = *localtime(&now);
            std::stringstream logfile;


            std::vector<char> data(argv[0].cbegin(), argv[0].cend());
            data.push_back('\0');
            const auto* base = basename(data.data());
            std::string base_str(base);

            utility::file::createDir(output_dir);


            utility::file::createDir(std::string(output_dir) + "/" + base_str);

            logfile << output_dir
                    << "/"
                    << base_str
                    << "/"
                    << std::put_time(&systemTime, "%Y%m%dT%H_%M_%S")
                    << ".nbs";

            // Close if required then open that file
            if (output_file.is_open()) {
                output_file.close();
            }

            log(logfile.str());

            output_file = std::ofstream(logfile.str());

            // Enable the streams we are after
            for (auto& setting : config["messages"].config) {
                // Lowercase the name
                std::string name = setting.first.as<std::string>();
                bool enabled = setting.second.as<bool>();

                if (handles.find(name) != handles.end()) {
                    auto& handle = handles[name];

                    if(enabled && !handle.enabled()) {
                        handle.enable();
                        log<NUClear::INFO>("Logging for", name, "enabled:");
                    }
                    else if(!enabled && handle.enabled()) {
                        handle.disable();
                        log<NUClear::INFO>("Logging for", name, "disabled:");
                    }
                }
                else {
                    log<NUClear::WARN>("This system does not know about the message type", name);
                }
            }
        });
    }
}
}
}
