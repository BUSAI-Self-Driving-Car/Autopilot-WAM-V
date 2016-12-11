#ifndef MODULE_GUIDANCE_PATHPLANNER_H
#define MODULE_GUIDANCE_PATHPLANNER_H

#include <nuclear>
#include "DStarLite.h"

namespace module {
namespace guidance {

    class PathPlanner : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the PathPlanner reactor.
        explicit PathPlanner(std::unique_ptr<NUClear::Environment> environment);
    private:
        DStarLite pathPlanner;
    };

}
}

#endif  // MODULE_GUIDANCE_PATHPLANNER_H
