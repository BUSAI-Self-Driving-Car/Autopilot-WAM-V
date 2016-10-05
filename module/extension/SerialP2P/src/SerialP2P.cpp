#include "SerialP2P.h"

#include "extension/Configuration.h"
#include "extension/P2P.h"

#include <boost/crc.hpp>

namespace module {
namespace extension {

    using ::extension::Configuration;
    using ::extension::P2P;
    using ::extension::P2PEmit;
    using ::extension::P2PListen;
    using ::NUClear::dsl::operation::Unbind;

    SerialP2P::SerialP2P(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment))
    , parseState(PARSE_STATE::WAIT){

        MESSAGE_START[0] = 0x17;
        MESSAGE_START[1] = 0xC0;
        MESSAGE_START[2] = 0x42;

        on<Configuration>("SerialP2P.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file SerialP2P.yaml

            // Connect to Radio
            std::string radioDevice = config["device"];
            unsigned int radioBaud = config["baud"];

            uart.close();
            uart.open(radioDevice, radioBaud);
        });

        on<Trigger<P2PListen>>().then([this](const P2PListen& value){
            auto hash = static_cast<uint32_t>(value.hash[0]);
            auto reaction = value.reaction;
            handlers.insert(std::make_pair(hash,reaction));
        });

        on<Trigger<Unbind<P2PListen>>>().then([this](const Unbind<P2PListen>& value){
            auto reactionId = value.reactionId;
            for (auto it = handlers.begin(); it != handlers.end(); ++it) {
               if (reactionId == it->second->reactionId) {
                   handlers.erase(it);
                   break;
               }
            }
        });

        on<Trigger<P2PEmit>>().then([this](const P2PEmit& value){
            //TODO: send over uart
        });

        on<IO,Priority::HIGH>(uart.native_handle(), IO::READ).then("Read", [this]{

            char c;
            auto bytes_read = uart.read(&c,1);

            if (bytes_read > 1) {
                throw std::logic_error("uart read more bytes than requested");
            }
            else if (bytes_read < 1 ) return;

            readbuffer.push_back(c);

            auto len = readbuffer.size();

            // Check if the last 3 bytes signifiy start of a new message
            if (len > 2) {

                if (readbuffer.at(len-3) == MESSAGE_START[0] &&  readbuffer.at(len-2) == MESSAGE_START[1]  &&  readbuffer.at(len-1) == MESSAGE_START[2])
                {
                    readbuffer.resize(0);
                    parseState = PARSE_STATE::TYPE;
                    return;
                }
            }
            if (len > 0) {

                switch (parseState) {

                case PARSE_STATE::WAIT :
                    return;

                case PARSE_STATE::TYPE :
                    if (len == 4) {
                        messageType = (*(reinterpret_cast<const uint32_t*>(readbuffer.data())));
                        readbuffer.resize(0);
                        parseState = PARSE_STATE::SIZE;
                    }
                    break;

                case PARSE_STATE::SIZE :
                    if (len == 4) {
                        messageSize = (*(reinterpret_cast<const uint32_t*>(readbuffer.data())));
                        readbuffer.resize(0);
                        readbuffer.reserve(messageSize);
                        parseState = PARSE_STATE::DATA;
                    }
                    break;

                case PARSE_STATE::DATA :
                    if (len == messageSize) {
                         messageData.resize(0);
                         messageData.reserve(len);
                         messageData.insert(messageData.begin(), readbuffer.begin(), readbuffer.end());
                         readbuffer.resize(0);
                         parseState = PARSE_STATE::CRC;
                    }
                    break;
                 case PARSE_STATE::CRC :
                    if (len == 4 &&  messageData.size() == messageSize) {
                        boost::crc_32_type crc32;
                        crc32.process_bytes(messageData.data(), messageData.size());
                        std::reverse(readbuffer.begin(), readbuffer.end());
                        auto checksum = (*(reinterpret_cast<const boost::crc_32_type::value_type*>(readbuffer.data())));

                        if (crc32.checksum() == checksum) {

                            // Using Trent magic emit the message to all on<P2P>
                            NUClear::dsl::store::ThreadStore<std::vector<char>>::value = &messageData;
                            auto range = handlers.equal_range(messageType);
                            for (auto it = range.first; it != range.second; ++it) {
                               auto task = it->second->getTask();
                               if (task) {
                                   powerplant.submit(std::move(task));
                               }
                            }
                            NUClear::dsl::store::ThreadStore<std::vector<char>>::value = nullptr;
                        }

                        readbuffer.resize(0);
                        parseState = PARSE_STATE::WAIT;
                    }
                    break;

                default:
                    log("Unknown PARSE STATE", parseState);
                    parseState = PARSE_STATE::WAIT;
                    //throw std::runtime_error("Unknown PARSE STATE");
                }
            }

        });

        on<Shutdown>().then([this]{
            uart.close();
        });
    }
}
}
