#ifndef MODULE_SUPPORT_LOGGING_VIDEOCOMPRESSOR_H
#define MODULE_SUPPORT_LOGGING_VIDEOCOMPRESSOR_H

#include <nuclear>

namespace module {
namespace support {
namespace logging {

    class VideoCompressor : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the VideoCompressor reactor.
        explicit VideoCompressor(std::unique_ptr<NUClear::Environment> environment);
    };

}
}
}

#endif  // MODULE_SUPPORT_LOGGING_VIDEOCOMPRESSOR_H
