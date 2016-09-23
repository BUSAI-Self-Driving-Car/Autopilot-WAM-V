#include "GCS.h"

#include "extension/Configuration.h"

namespace module {
namespace communication {

    using extension::Configuration;

    GCS::GCS(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
    {
        on<Configuration>("GCS.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file GCS.yaml
            try {
                // Connect to Radio
                std::string radioDevice = config["radio_device"].as<std::string>();
                unsigned int radioBaud = config["radion_baud"].as<unsigned int>();
                uart.close();
                uart.open(radioDevice, radioBaud);
                log<NUClear::INFO>("Open radio on device:", radioDevice," baud:", radioBaud);
            }
            catch(std::exception& ex) {
                log<NUClear::ERROR>("Failed to configure: ", ex.what());
            }
        });

        on<IO,Priority::HIGH>(uart.native_handle(), IO::READ).then("Read", [this]{

        });

        on<Shutdown>().then([this]{
            uart.close();
        });
    }
}
}
