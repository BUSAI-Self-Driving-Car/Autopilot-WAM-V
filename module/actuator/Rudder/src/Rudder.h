#ifndef MODULE_ACTUATOR_RUDDER_H
#define MODULE_ACTUATOR_RUDDER_H

#include <nuclear>
#include <utility/io/uart.h>
#include <deque>
#include <regex>

namespace module {
namespace actuator {

    class Rudder : public NUClear::Reactor {

        static constexpr size_t MAX_RESPONSE_LENGTH = 128;

        enum Side { PORT, STARBOARD };
        enum Direction { POSITIVE, NEGATIVE };

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

        template <enum Side, enum Direction> struct Limit {};

        template <enum Side>
        struct StepperPulse
        {
            StepperPulse(int c)
                : count(c)
            {}
            int count;
        };

        template <enum Side S>
        struct Stepper
        {
            using PulseType = StepperPulse<S>;
            static constexpr enum Side side = S;
            utility::io::uart uart;
            float theta_min, theta_max;
            ReactionHandle uart_handle, positive_home, negative_home;
            std::mutex command_mutex;
            std::deque<std::pair<std::string, bool>> command_queue;
            bool writing_command, homed, homing;
            uint8_t acceleration;
            uint16_t current_limit;
            uint32_t high_speed, low_speed;
            int pulse_min, pulse_max, target;
            float conversion;
            bool moving;

            std::string buffer;

            Stepper() : writing_command(false), homed(false), target(0), moving(false), homing(false) {}

            void queue_command(const std::string& command, bool critical = false)
            {
                std::lock_guard<std::mutex> lg(command_mutex);
                command_queue.push_back(std::make_pair(command, critical));
            }

            void move(float azimuth)
            {
                target = conversion*azimuth;
            }
        };

        Stepper<PORT> port;
        Stepper<STARBOARD> starboard;

        template <typename StepperType>
        void read_uart(StepperType& stepper)
        {
            for (int c = stepper.uart.get(); c >= 0; c = stepper.uart.get())
            {
                if (c == '\r')
                {
                    process(stepper);
                    stepper.buffer.clear();
                }
                else if (stepper.buffer.size() > MAX_RESPONSE_LENGTH)
                {
                    stepper.buffer.clear();
                    log<NUClear::ERROR>("Maximum response length exceeded");
                }
                else
                {
                    stepper.buffer.push_back(c);
                }
            }
        }

        template <typename StepperType>
        void write_command(StepperType& stepper)
        {
            std::lock_guard<std::mutex> lg(stepper.command_mutex);
            if (stepper.writing_command) return;
            if (stepper.command_queue.empty()) return;
            std::string command("@01"+stepper.command_queue.front().first+"\r");
            stepper.uart.write(command.c_str(), command.length());
            stepper.writing_command = true;

            emit(std::make_unique<NUClear::message::ServiceWatchdog<StepperType>>());
        }

        template <typename StepperType>
        void process(StepperType& stepper)
        {
            std::smatch match;

            // Based on motor response
            if (std::regex_search(stepper.buffer.cbegin(), stepper.buffer.cend(), match, std::regex("OK")))
            {
                std::lock_guard<std::mutex> lg(stepper.command_mutex);
                if (!stepper.command_queue.empty()) stepper.command_queue.pop_front();
            }
            else if (std::regex_search(stepper.buffer.cbegin(), stepper.buffer.cend(), match, std::regex(R"(\?(.+))")))
            {
                process_error(stepper, match.str(1));
                {
                    std::lock_guard<std::mutex> lg(stepper.command_mutex);
                    if (!stepper.command_queue.empty())
                    {
                        if (stepper.command_queue.front().second == false) 
                        { 
                            stepper.command_queue.pop_front(); 
                        }
                    }
                }
            }
            else if (std::regex_search(stepper.buffer.cbegin(), stepper.buffer.cend(), match, std::regex(R"((\d+))")))
            {
                int value = stoi(match.str(1));

                // Based on transmitted command
                std::string lc;
                {
                    std::lock_guard<std::mutex> lg(stepper.command_mutex);
                    if (stepper.command_queue.empty()) return;
                    lc = stepper.command_queue.front().first;
                }

                bool valid = false;
                if (std::regex_search(lc, std::regex("MST")))
                {
                    // motor status
                    process_status(stepper, value);
                    valid = true;
                }
                else if (std::regex_search(lc, std::regex("PX")))
                {
                    // motor status
                    emit(std::make_unique<StepperPulse<StepperType::side>>(value));
                    valid = true;
                }

                if (valid)
                {
                    std::lock_guard<std::mutex> lg(stepper.command_mutex);
                    if (!stepper.command_queue.empty()) 
                    {
                        stepper.command_queue.pop_front();
                    }
                }
                else
                {
                    log<NUClear::WARN>("Unexpected number return for stepper:", stepper.side, "command:", lc);
                }
            }

            bool empty = false;
            {
                std::lock_guard<std::mutex> lg(stepper.command_mutex);
                empty = stepper.command_queue.empty();
                stepper.writing_command = false;
            }
            
            if (empty)
            {
                if (stepper.homed) 
                {
                    stepper.queue_command((stepper.moving ? "T" : "X") + std::to_string(stepper.target), true); 
                }
                stepper.queue_command("MST");
                stepper.queue_command("PX");
            }

            write_command(stepper);
        }

        template <typename StepperType>
        void process_error(StepperType& stepper, const std::string& code)
        {
            log<NUClear::DEBUG>("Error:", code);

            if (code == "State Error")
            {
                std::lock_guard<std::mutex> lg(stepper.command_mutex);
                stepper.command_queue.push_front(std::make_pair("CLR", true));
            }
            else if (code == "Moving")
            {
                std::lock_guard<std::mutex> lg(stepper.command_mutex);
                if (stepper.command_queue.empty()) return;

                std::string& lc = stepper.command_queue.front().first;
                std::smatch match;
                if (std::regex_search(lc.cbegin(), lc.cend(), match, std::regex(R"(X(-\d+|\d+))")))
                {
                    lc = "T" + std::to_string(stepper.target);
                }
                std::cout << lc << std::endl;
            }
            else if (std::regex_search(code, std::regex("ABS/INC")))
            {
                std::lock_guard<std::mutex> lg(stepper.command_mutex);
                if (stepper.command_queue.empty()) return;

                std::string& lc = stepper.command_queue.front().first;
                std::smatch match;
                if (std::regex_search(lc.cbegin(), lc.cend(), match, std::regex(R"(T(-\d+|\d+))")))
                {
                    lc = "X" + std::to_string(stepper.target);
                }
            }
       }

        template <typename StepperType>
        void process_status(StepperType& stepper, const int& code)
        {
            if (code & MINUS_LIMIT_SWITCH)
            {
                emit(std::make_unique<Limit<PORT,NEGATIVE>>()); 
            }
            if (code & PLUS_LIMIT_SWITCH)
            {
                emit(std::make_unique<Limit<PORT,POSITIVE>>());
            }

            if ((code & CONSTANT_SPEED) ||
                (code & ACCELERATING) ||
                (code & DECELERATING))
            {
                stepper.moving = true;
            }
            else
            {
                stepper.moving = false;
            }
        }

        template <typename StepperType>
        void start(StepperType& stepper)
        {
            // Configure the stepper driver
            if (stepper.homing) return;
            stepper.homing = true;
            stepper.homed = false;
            stepper.queue_command("ABS", true);
            stepper.queue_command("ACC=" + std::to_string(stepper.acceleration), true);
            stepper.queue_command("DRVIC="+std::to_string(stepper.current_limit), true);
            stepper.queue_command("DRVMS=2", true);
            stepper.queue_command("HSPD="+std::to_string(stepper.high_speed), true);
            stepper.queue_command("LSPD="+std::to_string(stepper.low_speed), true);
            stepper.queue_command("RW", true);
            stepper.queue_command("EO=1",true);

            // Go to negative limit
            stepper.queue_command("L-", true);

            stepper.negative_home.enable();
        }

        template <typename StepperType>
        void on_watchdog(StepperType& stepper)
        {
            log<NUClear::WARN>("Stepper transmit timeout on side:", stepper.side);
            stepper.writing_command = false;
            write_command(stepper);
        }

        template <typename StepperType>
        void negative_limit(StepperType& stepper)
        {
            log<NUClear::INFO>("Negative limit reached on side:", stepper.side);
            stepper.queue_command("J+", true);

            stepper.negative_home.disable();
            stepper.positive_home.enable();
        }

        template <typename StepperType>
        void positive_limit(StepperType& stepper, const typename StepperType::PulseType& pulse)
        {
            log<NUClear::INFO>("Positive limit reached on side:", stepper.side);
            stepper.pulse_min = -pulse.count/2;
            stepper.pulse_max = pulse.count/2;
            stepper.conversion = pulse.count / (stepper.theta_max - stepper.theta_min);

            stepper.target = 0;
            stepper.queue_command("PX=" + std::to_string(int(pulse.count/2)), true);

            stepper.positive_home.disable();
            stepper.homed = true;
            stepper.homing = false;
        }

    public:
        /// @brief Called by the powerplant to build and setup the Rudder reactor.
        explicit Rudder(std::unique_ptr<NUClear::Environment> environment);
    };

    template <enum Rudder::Side S> constexpr Rudder::Side Rudder::Stepper<S>::side;
}
}

#endif  // MODULE_ACTUATOR_RUDDER_H
