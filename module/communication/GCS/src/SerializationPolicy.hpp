#ifndef MODULE_COMMUNICATION_SERIALIZATIONPOLICY_HPP
#define MODULE_COMMUNICATION_SERIALIZATIONPOLICY_HPP

namespace module {
namespace communication {

struct SerializationPolicy {

    enum {
        INVALID_MESSAGE = 0,
        PROTOBUFF,
        END_OF_MESSAGES
    };

    enum { NUMBER_OF_MESSAGES = END_OF_MESSAGES - 1 };
    enum { MAX_MESSAGE_SIZE = 128 };


};

}
}

#endif // MODULE_COMMUNICATION_SERIALIZATIONPOLICY_HPP
