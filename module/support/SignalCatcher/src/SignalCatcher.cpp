#include "SignalCatcher.h"
#include <csignal>

namespace module {
    namespace support {

        // Set our initial shutdown request state
        volatile bool SignalCatcher::userRequestedShutdown = false;

        // Initialize our powerplant variable
        NUClear::PowerPlant* SignalCatcher::POWER_PLANT = nullptr;

        // Our segmentation fault converter function
        void sigsegv(int sig) {

            throw std::runtime_error("Segmentation Fault");
        }

        void sigabrt(int sig) {

            throw std::runtime_error("Abort signal");
        }

        SignalCatcher::SignalCatcher(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            // Store our powerplant in the static variable
            POWER_PLANT = &powerplant;
            struct sigaction action;

            // Setup our segmentation fault signal handler/converter
            std::memset(&action, 0, sizeof(action));
            action.sa_handler = sigsegv;
            action.sa_flags = SA_NODEFER;
            sigaction(SIGSEGV, &action, nullptr);

            // Setup our abort signal handler/converter
            std::memset(&action, 0, sizeof(action));
            action.sa_handler = sigabrt;
            action.sa_flags = SA_NODEFER;
            sigaction(SIGABRT, &action, nullptr);

            // On sigint run the sigint handler
            std::signal(SIGINT, &SignalCatcher::sigintHandler);
        }

        void SignalCatcher::sigintHandler(int) {

            // Output that a shutdown command was sent (so the user knows the ctrl-c worked)
            std::cout << std::endl << "Shutdown Command Sent" << std::endl;

            // If this is the first time they asked
            if(!userRequestedShutdown) {

                // Ask the system to shutdown, and flag that the user has asked once
                POWER_PLANT->shutdown();
                userRequestedShutdown = true;
            }
            // If this is the second time, kill everything
            else {
                exit(1);
            }
        }

    }  // support
}  // modules
