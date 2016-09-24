#include "Torqeedo.h"

#include "extension/Configuration.h"

namespace module {
namespace actuator {

    using extension::Configuration;

    Torqeedo::Torqeedo(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
    {

        on<Configuration>("Torqeedo.yaml").then([this] (const Configuration& config)
        {
            // Use configuration here from file Torqeedo.yaml
            std::string name = "test";

            hal.reset(new TorqeedoHAL(name, uart));
            uart.open("/dev/ttyS0", 19200);
            i = 0;
            started = false;
        });

        on<IO,Priority::HIGH>(uart.native_handle(), IO::READ).then("Read", [this]
        {
            hal->read();
        });

        on<Every<1, std::chrono::seconds>, Sync<int>>().then([this] ()
        {
            if (!started)
            {
                std::cout << "Motor Start" << std::endl;
                hal->speed(0);
                hal->start();
                std::cout << "GO" << std::endl;
                started = true;
            }
        });

        on<Every<1, std::chrono::seconds>, Sync<int>>().then([this] ()
        {
            if (started)
            {
                i = (i + 1) % 25;
                float speed = sin(M_PI*double(i)/25.0)/4;
                std::cout << i << ", " << speed << std::endl;
                hal->speed(speed);
            }
        });
    }
}
}
