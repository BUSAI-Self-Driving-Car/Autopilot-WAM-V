#ifndef MODULE_CONTROL_PID_H
#define MODULE_CONTROL_PID_H

#include <nuclear>
#include "opengnc/control/pid.hpp"

namespace module {
namespace control {

    class PID : public NUClear::Reactor
    {
    private:
        struct CascadeController
        {
            opengnc::control::pid<double> velocity;
            opengnc::control::pid<double> position;
        };

        CascadeController surge, sway, yaw;
        NUClear::clock::time_point last_velocity;
        NUClear::clock::time_point last_position;

    public:
        /// @brief Called by the powerplant to build and setup the PID reactor.
        explicit PID(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_CONTROL_PID_H
