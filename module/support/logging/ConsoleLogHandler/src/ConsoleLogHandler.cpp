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

#include "ConsoleLogHandler.h"

#include "utility/strutil/ansi.h"
#include "utility/support/evil/pure_evil.h"

namespace module {
    namespace support {
        namespace logging {

            using NUClear::message::LogMessage;
            using NUClear::message::ReactionStatistics;
            using utility::strutil::Colour;

            ConsoleLogHandler::ConsoleLogHandler(std::unique_ptr<NUClear::Environment> environment)
             : Reactor(std::move(environment)), mutex() {
                on<Trigger<ReactionStatistics>>().then([this](const ReactionStatistics & stats) {
                    if (stats.exception) {

                        std::lock_guard<std::mutex> lock(mutex);

                        // Get our reactor name
                        std::string reactor = stats.identifier[1];

                        // Strip to the last semicolon if we have one
                        size_t lastC = reactor.find_last_of(':');
                        reactor = lastC == std::string::npos ? reactor : reactor.substr(lastC + 1);

#if !defined(NDEBUG) && !defined(__APPLE__) // We have a cold hearted monstrosity that got built!

                        // Print our exception detals
                        std::cerr << reactor << " "
                                  << (stats.identifier[0].empty() ? "" : "- " + stats.identifier[0] + " ")
                                  << Colour::brightred << "Exception:" << " "
                                  << Colour::brightred << utility::support::evil::exception_name
                                  << std::endl;

                        // Print our stack trace
                        for (auto& s : utility::support::evil::stack) {
                            std::cerr << "\t" << Colour::brightmagenta << s.file
                                      << ":" << Colour::brightmagenta << s.lineno
                                      << " " << s.function
                                      << std::endl;
                        }
#else
                        try {
                            std::rethrow_exception(stats.exception);
                        }
                        catch (const std::exception& ex) {

                            std::string exceptionName = NUClear::util::demangle(typeid(ex).name());

                            std::cerr << reactor << " "
                                      << (stats.identifier[0].empty() ? "" : "- " + stats.identifier[0] + " ")
                                      << Colour::brightred << "Exception:" << " "
                                      << Colour::brightred << exceptionName << " "
                                      << ex.what()
                                      << std::endl;
                        }
                        // We don't actually want to crash
                        catch (...) {

                            std::cerr << reactor << " "
                                      << (stats.identifier[0].empty() ? "" : "- " + stats.identifier[0] + " ")
                                      << Colour::brightred << "Exception of unkown type"
                                      << std::endl;
                        }
#endif
                    }
                });


                on<Trigger<LogMessage>>().then([this] (const LogMessage& message) {

                    std::lock_guard<std::mutex> lock(mutex);

                    // Where this message came from
                    std::string source = "";

                    // If we know where this log message came from, we display that
                    if (message.task) {
                        // Get our reactor name
                        std::string reactor = message.task->identifier[1];

                        // Strip to the last semicolon if we have one
                        size_t lastC = reactor.find_last_of(':');
                        reactor = lastC == std::string::npos ? reactor : reactor.substr(lastC + 1);

                        // This is our source
                        source = reactor + " " + (message.task->identifier[0].empty() ? "" : "- " + message.task->identifier[0] + " ");
                    }

                    // Output the level
                    switch(message.level) {
                        case NUClear::TRACE:
                            std::cerr << source << "TRACE: ";
                            break;
                        case NUClear::DEBUG:
                            std::cerr << source << Colour::green << "DEBUG: ";
                            break;
                        case NUClear::INFO:
                            std::cerr << source << Colour::brightblue << "INFO: ";
                            break;
                        case NUClear::WARN:
                            std::cerr << source << Colour::yellow << "WARN: ";
                            break;
                        case NUClear::ERROR:
                            std::cerr << source << Colour::brightred << "ERROR: ";
                            break;
                        case NUClear::FATAL:
                            std::cerr << source << Colour::brightred << "FATAL: ";
                            break;
                    }

                    // Output the message
                    std::cerr << message.message << std::endl;
                });
            }

        }  // logging
    }  // support
}  // modules
