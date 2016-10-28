#include "GPS.h"

#include "extension/Configuration.h"

using namespace module::sensor;
using extension::Configuration;

GPS::GPS(std::unique_ptr<NUClear::Environment> environment)
: Reactor(std::move(environment))
{

    on<Configuration>("GPS.yaml").then([this] (const Configuration& config)
    {
        // Use configuration here from file GPS.yaml
        uart.open(config["device"].as<std::string>(), config["baud"].as<unsigned int>());

        uart_handle.unbind();
        uart_handle = on<IO,Priority::HIGH>(uart.native_handle(), IO::READ).then("gps read", [this]
        {
            for (int c = uart.get(); c >= 0; c = uart.get())
            {
                if (c == '\r') {}
                else if (c == '\n')
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
        });
    });
}

void GPS::process()
{
    auto tokens = split(buffer, ',');

    if (tokens[0] == "$GPVTG")
    {
        // Velocity made good
        double track = std::numeric_limits<double>::quiet_NaN();
        double velocity = std::numeric_limits<double>::quiet_NaN();
        try
        {
            track = M_PI / 180.0 * std::stod(tokens[1]);
            velocity = 1000.0 / 3600.0 * std::stod(tokens[7]);
        }
        catch (std::invalid_argument&) {}

        state.velocity << velocity*std::cos(track), velocity*std::sin(track);
    }
    else if (tokens[0] == "$GPGGA")
    {
        // Essential fix data
        state.lla[0] = std::numeric_limits<double>::quiet_NaN();
        state.lla[1] = std::numeric_limits<double>::quiet_NaN();
        state.fix_type = message::sensor::GPSRaw::FixType::NO_FIX;

        try
        {
            state.lla[0] = std::stod(tokens[2]);
            if (tokens[3] == "S") { state.lla[0] *= -1; }
            state.lla[1] = std::stod(tokens[4]);
            if (tokens[5] == "W") { state.lla[1] *= -1; }
            state.fix_type = std::stoi(tokens[6]);
            state.hdop = std::stod(tokens[8]);
            state.lla[2] = std::stod(tokens[9]) + std::stod(tokens[10]);
        }
        catch (std::invalid_argument&) {}
    }
    else if (tokens[0] == "$GPGSA")
    {
        // GPS DOP and active satellites
        state.pdop = std::numeric_limits<double>::quiet_NaN();
        state.hdop = std::numeric_limits<double>::quiet_NaN();
        state.vdop = std::numeric_limits<double>::quiet_NaN();

        try
        {
            state.pdop = std::stod(tokens[15]);
            state.hdop = std::stod(tokens[16]);
            state.vdop = std::stod(tokens[17]);
        }
        catch (std::invalid_argument&) {}
    }
    else if (tokens[0] == "$GPGSV")
    {
        // Satellites in view
        uint sentences = 0;
        uint seq = 0;
        uint satellites = 0;

        try
        {
            sentences = std::stoi(tokens[1]);
            seq = std::stoi(tokens[2]);
            satellites = std::stoi(tokens[3]);
            state.satellites.resize(satellites);
            int start = seq*4;
            int end = std::min((seq+1)*4, satellites);

            for (int i = start; i < end; ++i)
            {
                int index = 4 + (i-start)*4;
                state.satellites[i].PRN = std::stoi(tokens[index+0]);
                state.satellites[i].elevation = std::stoi(tokens[index+1]);
                state.satellites[i].azimuth = std::stoi(tokens[index+2]);
                state.satellites[i].SNR = std::stoi(tokens[index+3]);
            }
        }
        catch (std::invalid_argument&) {}
    }
    else if (tokens[0] == "$GPGLL")
    {
        // Geographic lattitude and and longitude
        emit(std::make_unique<message::sensor::GPSRaw>(state));
    }

}
