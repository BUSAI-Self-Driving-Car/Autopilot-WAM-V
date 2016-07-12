#include "NetworkConfiguration.h"

#include "extension/Configuration.h"

namespace module {
namespace support {
namespace configuration {

    using extension::Configuration;

    NetworkConfiguration::NetworkConfiguration(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("NetworkConfiguration.yaml").then([this] (const Configuration& config) {
            auto netConfig = std::make_unique<NUClear::message::NetworkConfiguration>();
            netConfig->name = config["name"].as<std::string>();
            netConfig->multicastGroup = config["address"].as<std::string>();
            netConfig->multicastPort = config["port"].as<int>();
            emit<Scope::DIRECT>(netConfig);
        });
    }
}
}
}
