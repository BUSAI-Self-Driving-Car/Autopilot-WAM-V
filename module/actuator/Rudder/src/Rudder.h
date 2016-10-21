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

        template <enum Side S>
        struct Stepper
        {
            static constexpr enum Side side = S;
            utility::io::uart uart;
            float theta_min, theta_max;
            ReactionHandle uart_handle, positive_home, negative_home;
            std::mutex command_mutex;
            std::deque<std::pair<std::string, bool>> command_queue;
            uint8_t acceleration;
            uint16_t current_limit;
            uint32_t high_speed, low_speed;
            int pulse_min, pulse_max, target;
            float conversion;

            std::string buffer;

            void queue_command(std::string command, bool critical = false)
            {
                std::lock_guard<std::mutex> lg(command_mutex);
                command_queue.push_back(std::make_pair(command, critical));
            }
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
            std::string command("@01"+stepper.command_queue.front().first+"\r");
            log<NUClear::DEBUG>(int(stepper.side), "TX:", command);
            stepper.uart.write(command.c_str(), command.length());

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
                        if (stepper.command_queue.front().second == false) { stepper.command_queue.pop_front(); }
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
                    if (!stepper.command_queue.empty()) stepper.command_queue.pop_front();
                }
                else
                {
                    log<NUClear::WARN>("Unexpected number return for stepper:", stepper.side, "command:", lc);
                }
            }

            {
                std::lock_guard<std::mutex> lg(stepper.command_mutex);
                if (stepper.command_queue.empty())
                {
                    stepper.queue_command("MST");
                    stepper.queue_command("PX");
                }
            }
            write_command(stepper);
        }

        template <typename StepperType>
        void process_error(StepperType& config, const std::string& code)
        {

        }

        template <typename StepperType>
        void process_status(StepperType& config, const int& code)
        {
            if (code & MINUS_LIMIT_SWITCH)
            {

            }
        }

    public:
        /// @brief Called by the powerplant to build and setup the Rudder reactor.
        explicit Rudder(std::unique_ptr<NUClear::Environment> environment);
    };

    template <enum Rudder::Side S> constexpr Rudder::Side Rudder::Stepper<S>::side;
}
}

#endif  // MODULE_ACTUATOR_RUDDER_H
