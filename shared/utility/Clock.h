#ifndef UTILITY_CLOCK_H
#define UTILITY_CLOCK_H

#include <cstdint>
#include <nuclear_bits/clock.hpp>
#include <chrono>

namespace utility {
    struct Clock {

        static inline uint64_t ToMilli(const NUClear::clock::time_point& time_point)
        {
            using namespace std::chrono;
            return   duration_cast<duration<uint64_t, std::milli>>(time_point.time_since_epoch()).count();
        }

        static inline double ToMilli(const NUClear::clock::duration& d)
        {
            using namespace std::chrono;
            return duration_cast<duration<double, std::milli>>(d).count();
        }
    };
}

#endif // UTILITY_CLOCK_H
