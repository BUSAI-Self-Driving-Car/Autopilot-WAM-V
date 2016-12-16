#include "LightTower.h"
#include <regex>

#include "extension/Configuration.h"
#include "message/status/EStop.h"
#include "message/status/Mode.h"
#include "message/propulsion/PropulsionStatus.h"

namespace module {
namespace actuator {

    using extension::Configuration;
    using message::status::EStop;
    using message::status::Mode;
    using message::propulsion::PropulsionStatus;

    LightTower::LightTower(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("LightTower.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file LightTower.yaml
            std::string device = config["device"].as<std::string>();
            unsigned int baud = config["baud"].as<unsigned int>();

            log<NUClear::INFO>("Connecting to Light Tower", device, "with baud", baud);
            uart.open(device, baud);
        });

        on<IO,Priority::LOW>(uart.native_handle(), IO::READ).then("light tower read", [this]
        {
            if (uart.good())
            {
                for (int c = uart.get(); c > 0; c = uart.get())
                {
                    if (c == '\n')
                    {
                        process();
                        buffer.clear();
                    }
                    else if (buffer.size() > MAX_RESPONSE_LENGTH)
                    {
                        buffer.clear();
                        log<NUClear::ERROR>("Maximum response length exceeded");
                    }
                    else
                    {
                        buffer.push_back(c);
                    }
                }
            }
        });

        on<Every<500, std::chrono::milliseconds>, Network<Mode>, Network<PropulsionStatus>>().then([this] (const Mode& mode,
                                                                                                   const PropulsionStatus& status)
        {
            if (status.enabled)
            {
                switch (int(mode.type))
                {
                    case Mode::Type::MANUAL:
                    {
                        const std::string command("light yellow\n");
                        uart.write(command.c_str(), command.length());
                    }
                    break;
                    case Mode::Type::AUTONOMOUS:
                    {
                        const std::string command("light green\n");
                        uart.write(command.c_str(), command.length());
                    }
                    break;
                }
            }
            else
            {
                const std::string command("light home\n");
                uart.write(command.c_str(), command.length());
            }
        });
    }

    void LightTower::process()
    {
        std::smatch match;

        // Based on motor response
        if (std::regex_search(buffer.cbegin(), buffer.cend(), match, std::regex("ON")))
        {
            emit(std::make_unique<EStop>(NUClear::clock::now(), true));
        }
        else if (std::regex_search(buffer.cbegin(), buffer.cend(), match, std::regex("OFF")))
        {
            emit(std::make_unique<EStop>(NUClear::clock::now(), false));
        }
    }
}
}
