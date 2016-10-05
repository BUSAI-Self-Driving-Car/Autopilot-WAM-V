#ifndef MESSAGE_MESSAGEBASE_H
#define MESSAGE_MESSAGEBASE_H

#include <nuclear>

namespace message {
    class MessageBase {
    };
}

namespace NUClear {
    namespace util {
        namespace serialise {

            template <typename T>
            struct Serialise<T, std::enable_if_t<std::is_base_of<::message::MessageBase, T>::value, T>> {

                using protobuf_type = typename T::protobuf_type;

                static inline std::vector<char> serialise(const T& in) {

                    protobuf_type proto = in;

                    std::vector<char> output(proto.ByteSize());
                    proto.SerializeToArray(output.data(), output.size());

                    return output;
                }

                static inline T deserialise(const std::vector<char>& in) {

                    // Make a buffer
                    protobuf_type out;

                    // Deserialize it
                    out.ParseFromArray(in.data(), in.size());
                    return out;
                }

                static inline std::array<uint64_t, 2> hash() {

                    // We have to construct an instance to call the reflection functions
                    protobuf_type type;

                    // We have to remove the 'protobuf' namespace
                    std::string typeName = type.GetTypeName().substr(9);

                    // We base the hash on the name of the protocol buffer, removing the protobuf prefix on typeName
                    return murmurHash3(typeName.c_str(), typeName.size());
                }
            };

        }
    }
}

#endif  // MESSAGE_MESSAGEBASE_H
