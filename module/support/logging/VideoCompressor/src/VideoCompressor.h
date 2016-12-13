#ifndef MODULE_SUPPORT_LOGGING_VIDEOCOMPRESSOR_H
#define MODULE_SUPPORT_LOGGING_VIDEOCOMPRESSOR_H

#include <nuclear>
#include <x264.h>
#include <libswscale/swscale.h>

namespace module {
namespace support {
namespace logging {

    class VideoCompressor : public NUClear::Reactor {
    private:
        std::unique_ptr<SwsContext> swsContext;
        std::unique_ptr<x264_t, std::function<void(x264_t*)>> encoder;
        x264_param_t param;

    public:
        /// @brief Called by the powerplant to build and setup the VideoCompressor reactor.
        explicit VideoCompressor(std::unique_ptr<NUClear::Environment> environment);
    };

}
}
}

#endif  // MODULE_SUPPORT_LOGGING_VIDEOCOMPRESSOR_H
