#ifndef MODULE_ACTUATOR_STEPPER_H
#define MODULE_ACTUATOR_STEPPER_H

#include "utility/io/uart.h"
#include <string>
#include <regex>
#include <vector>
#include <mutex>
#include <deque>
#include <future>

namespace module {
namespace actuator {

    class Stepper
    {
        enum MOTOR_STATUS
        {
            CONSTANT_SPEED = 1 << 0,
            ACCELERATING = 1 << 1,
            DECELERATING = 1 << 2,
            HOME_INPUT_SWITCH = 1 << 3,
            MINUS_LIMIT_SWITCH = 1 << 4,
            PLUS_LIMIT_SWITCH = 1 << 5,
            MINUS_LIMIT_ERROR = 1 << 6,
            PLUS_LIMIT_ERROR = 1 << 7,
            LATCH_INPUT_STATUS = 1 << 8,
            Z_INDEX_STATUS = 1 << 9,
            TOC_TIME_OUT_STATUS = 1 << 10
        };

        enum
        {
            UNINITIALISED,
            MOVING_TO_MINIMUM_LIMIT,
            MOVING_TO_MAXIMUM_LIMIT,
            MOVING_TO_CENTRE,
            INITIALISED
        } state;

        int get_current_position();
        void process(std::string,std::string);
        void command(std::string, bool critical);
        void statGrab(std::string);
        void error(std::string); // too be added at some point
        void write_command();
        void write_command(std::string);

    public:
        Stepper(utility::io::uart& rs485, float thetaMax, float thetaMin);

        void command_sent();
        void run();
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
        std::mutex command_mtx;
        std::deque<std::pair<std::string, bool>> command_queue;
        bool writing, homed, motorMoving, posRequest;
        std::future<void> future;

        // rad to pulse
        float conv = 1; // to be decided

        // max & min positions
        const float thetaMax;
        const float thetaMin;

        // High & Low Speed for driver in PPS
        uint HSPD = 40000;
        uint HSPD_HOME = 25000;
        uint LSPD = 10000;

        // variable to express what was the last rs485 command, possibly string or other???
        std::string lastCommand;
        std::string pref;
        std::chrono::high_resolution_clock::time_point timeSent;
        std::condition_variable stop_cond_var, pos_cond_var;
        std::mutex stop_mtx, pos_mtx;
        bool limit;
        int pulsePos, pulseMax, pulseMin, targetPos;

        // Encoder Position
        int posEnc;
        // Encoder Bearing Position
        float encBearing;
    };

}
}

#endif  // MODULE_ACTUATOR_STEPPER_H
