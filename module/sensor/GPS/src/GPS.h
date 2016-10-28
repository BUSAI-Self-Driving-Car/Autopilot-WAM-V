#ifndef MODULE_SENSOR_GPS_H
#define MODULE_SENSOR_GPS_H

#include <nuclear>
#include <utility/io/uart.h>
#include "message/sensor/GPSRaw.h"

namespace module {
namespace sensor {

    class GPS : public NUClear::Reactor
    {
    private:
        static constexpr int MAX_RESPONSE_LENGTH = 256;
        utility::io::uart uart;
        ReactionHandle uart_handle;
        std::string buffer;
        message::sensor::GPSRaw state;

        void process();

        inline std::vector<std::string> split(const std::string& str, char delimeter)
        {
            std::vector<std::string> elements;
            std::stringstream ss(str);
            std::string item;
            while (std::getline(ss, item, delimeter))
            {
                elements.push_back(item);
            }
            return elements;
        }

    public:
        /// @brief Called by the powerplant to build and setup the GPS reactor.
        explicit GPS(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_SENSOR_GPS_H
