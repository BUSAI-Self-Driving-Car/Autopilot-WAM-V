#ifndef MODULE_SENSOR_DTO_SERIALIZATION_POLICY_HPP
#define MODULE_SENSOR_DTO_SERIALIZATION_POLICY_HPP

#include <cstddef>
#include <datatransfer/binary_serialization.hpp>
#include "imu_measurement.hpp"

namespace module {
namespace sensor {
namespace dto {

struct serialization_policy
{
    enum
    {
        INVALID_MESSAGE = 0,
        IMU_MEASUREMENT,
        END_OF_MESSAGES
    };

    template <int N>
    struct data
    {
        using type = void;
        static constexpr size_t length = 0;
    };

    enum { NUMBER_OF_MESSAGES = END_OF_MESSAGES - 1 };
    enum { MAX_MESSAGE_SIZE = 128 };

    template <typename input_output_stream>
    struct serialization
    {
        using write_policy = datatransfer::binary_serialization::write_policy<input_output_stream>;
        using read_policy = datatransfer::binary_serialization::read_policy<uint8_t, MAX_MESSAGE_SIZE>;
        using checksum_policy = datatransfer::binary_serialization::checksum_policy;
        using size_policy = datatransfer::binary_serialization::size_policy;
    };

    static constexpr bool valid(int N) { return (N > 0) && (N < END_OF_MESSAGES); }
};

template <>
struct serialization_policy::data<serialization_policy::IMU_MEASUREMENT>
{
    using type = imu_measurement;
    static constexpr size_t length = sizeof(type);
};

}
}
}

#endif // MODULE_SENSOR_DTO_SERIALIZATION_POLICY_HPP
