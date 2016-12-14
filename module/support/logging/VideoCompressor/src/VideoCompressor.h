#ifndef MODULE_SUPPORT_LOGGING_VIDEOCOMPRESSOR_H
#define MODULE_SUPPORT_LOGGING_VIDEOCOMPRESSOR_H

#include <nuclear>
#include <x264.h>

namespace module {
namespace support {
namespace logging {

    class VideoCompressor : public NUClear::Reactor {
    private:

        struct Encoder {
            std::unique_ptr<x264_t, std::function<void(x264_t*)>> encoder;
            x264_nal_t* nals;
            std::unique_ptr<x264_picture_t, std::function<void(x264_picture_t*)>> inputPicture;
            x264_picture_t outputPicture;
        };

        x264_param_t compressionParams;

        Encoder rEncoder;
        Encoder g1Encoder;
        Encoder g2Encoder;
        Encoder bEncoder;

    public:
        /// @brief Called by the powerplant to build and setup the VideoCompressor reactor.
        explicit VideoCompressor(std::unique_ptr<NUClear::Environment> environment);
    };

}
}
}

#endif  // MODULE_SUPPORT_LOGGING_VIDEOCOMPRESSOR_H
