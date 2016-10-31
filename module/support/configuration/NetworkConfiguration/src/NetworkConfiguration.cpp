/*
 * Copyright (C) 2013-2016 Trent Houliston <trent@houliston.me>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "NetworkConfiguration.h"

#include "extension/Configuration.h"

namespace module {
namespace support {
namespace configuration {

    using extension::Configuration;

    NetworkConfiguration::NetworkConfiguration(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("NetworkConfiguration.yaml").then([this] (const Configuration& config) {
            auto netConfig = std::make_unique<NUClear::message::NetworkConfiguration>();
            netConfig->name = config["name"];
            netConfig->multicastGroup = config["address"];
            netConfig->multicastPort = config["port"];
            emit<Scope::DIRECT>(netConfig);
        });

        on<Trigger<NUClear::message::NetworkJoin>>().then([this](const NUClear::message::NetworkJoin& message){
            char str[INET_ADDRSTRLEN];
            uint32_t addr = htonl(message.address);
            inet_ntop(AF_INET, &addr, str, INET_ADDRSTRLEN);
            log<NUClear::INFO>("Connected to", message.name, "on", str);
        });

        on<Trigger<NUClear::message::NetworkLeave>>().then([this](const NUClear::message::NetworkLeave& message){
            char str[INET_ADDRSTRLEN];
            uint32_t addr = htonl(message.address);
            inet_ntop(AF_INET, &addr, str, INET_ADDRSTRLEN);
            log<NUClear::INFO>("Disconnected from", message.name, "on", str);
        });
    }
}
}
}
