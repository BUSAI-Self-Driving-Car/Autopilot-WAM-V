#ifndef MODULE_ACTUATOR_STEPPER_H
#define MODULE_ACTUATOR_STEPPER_H

#include "utility/io/uart.h"
#include <string>
#include <regex>
#include <vector>

namespace module {
namespace actuator {

    class Stepper {

        void process(std::string,std::string);
        void command(std::string,std::string);
        void statGrab(std::string);
        void error(std::string); // too be added at some point

    public:
        Stepper(utility::io::uart& rs485);

        void begin();    // Takes address of motor driver (will be either 01 or 02)
        bool isHomed() const { return homed; }
        void motorHome();       // Homes the motor using limit switches & centres the encoder
        void motorEnable(bool); // Enable/Disable motor
        void motorStop(bool);   // Stop motor using deceleration (true) or crash stop (false)
        void azimuth(float);    // Make motor move to somewhere
        void motorStatus();     // Thread to check motor status
        void motorPos();        // Thread to check motor encoder position
        void read();            // Read comms bus
        float get_azimuth() const { return encBearing; }

    private:        
        utility::io::uart& rs485;

        std::string buffer;

        bool homed;

        // rad to pulse
        const float conv = 1; // to be decided

        // max & min positions
        const float thetaMax = 1; // +pi/2
        const float thetaMin = 1; // -pi/2

        // High & Low Speed for driver in PPS
        int HSPD = 10000;
        int LSPD = 1000;

        // variable to express what was the last rs485 command, possibly string or other???
        std::string lastCommand;

        int pulsePos;

        // motor status variables
        bool motorMoving = false;
        bool minLim = false;
        bool minLimErr = false;
        bool maxLim = false;
        bool maxLimErr = false;

        // Encoder Position
        int posEnc;
        // Encoder Bearing Position
        float encBearing;
    };

}
}

#endif  // MODULE_ACTUATOR_STEPPER_H
