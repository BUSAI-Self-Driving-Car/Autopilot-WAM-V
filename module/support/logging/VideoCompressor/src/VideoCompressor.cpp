#include "VideoCompressor.h"

#include "extension/Configuration.h"
#include "message/vision/Image.h"
#include "message/vision/CompressedImage.h"

namespace module {
namespace support {
namespace logging {

    using extension::Configuration;
    using message::vision::Image;
    using message::vision::CompressedImage;

    VideoCompressor::VideoCompressor(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("VideoCompressor.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file VideoCompressor.yaml

            // This is the width of the image.
            // Eventually it should be adapted as the image changes and resetting the compressor
            int width  = config["width"];
            int height = config["height"];

            // Setup our settings
            x264_param_default_preset(&compressionParams, "ultrafast", "zerolatency");
            compressionParams.i_threads = X264_THREADS_AUTO;
            compressionParams.i_width = width / 2;
            compressionParams.i_height = height / 2;
            compressionParams.i_fps_num = config["fps"];
            compressionParams.i_fps_den = 1;
            // Intra refresh:
            compressionParams.i_keyint_max = 30;
            compressionParams.b_intra_refresh = 1;
            //Rate control:
            compressionParams.rc.i_rc_method = X264_RC_CRF;
            compressionParams.rc.f_rf_constant = config["crf"];
            compressionParams.rc.f_rf_constant_max = config["crf_max"];

            //For streaming:
            compressionParams.b_repeat_headers = 1;
            compressionParams.b_annexb = 1;
            x264_param_apply_profile(&compressionParams, "baseline");

            // Create our encoders using these settings
            rEncoder.encoder = std::unique_ptr<x264_t, std::function<void(x264_t*)>>(x264_encoder_open(&compressionParams), x264_encoder_close);
            g1Encoder.encoder = std::unique_ptr<x264_t, std::function<void(x264_t*)>>(x264_encoder_open(&compressionParams), x264_encoder_close);
            g2Encoder.encoder = std::unique_ptr<x264_t, std::function<void(x264_t*)>>(x264_encoder_open(&compressionParams), x264_encoder_close);
            bEncoder.encoder = std::unique_ptr<x264_t, std::function<void(x264_t*)>>(x264_encoder_open(&compressionParams), x264_encoder_close);
            if (!(rEncoder.encoder && g1Encoder.encoder && g2Encoder.encoder && bEncoder.encoder)) {
                log<NUClear::ERROR>("The x264 encoder failed to open");
            }

            // Allocate pictures to copy bytes into
            std::function<void(x264_picture_t*)> picture_delete = [] (x264_picture_t* ptr) {
                x264_picture_clean(ptr);
                delete ptr;
            };
            rEncoder.inputPicture = std::unique_ptr<x264_picture_t, std::function<void(x264_picture_t*)>>(new x264_picture_t(), picture_delete);
            g1Encoder.inputPicture = std::unique_ptr<x264_picture_t, std::function<void(x264_picture_t*)>>(new x264_picture_t(), picture_delete);
            g2Encoder.inputPicture = std::unique_ptr<x264_picture_t, std::function<void(x264_picture_t*)>>(new x264_picture_t(), picture_delete);
            bEncoder.inputPicture = std::unique_ptr<x264_picture_t, std::function<void(x264_picture_t*)>>(new x264_picture_t(), picture_delete);

            x264_picture_alloc(rEncoder.inputPicture.get(), X264_CSP_I420, width / 2, height / 2);
            x264_picture_alloc(g1Encoder.inputPicture.get(), X264_CSP_I420, width / 2, height / 2);
            x264_picture_alloc(g2Encoder.inputPicture.get(), X264_CSP_I420, width / 2, height / 2);
            x264_picture_alloc(bEncoder.inputPicture.get(), X264_CSP_I420, width / 2, height / 2);

            // Zero out the bytes so we can just fill the Y plane
            std::memset(rEncoder.inputPicture->img.plane[0], 128, (width * height) / 2);
            std::memset(g1Encoder.inputPicture->img.plane[0], 128, (width * height) / 2);
            std::memset(g2Encoder.inputPicture->img.plane[0], 128, (width * height) / 2);
            std::memset(bEncoder.inputPicture->img.plane[0], 128, (width * height) / 2);
        });

        on<Trigger<Image>, Buffer<4>, Sync<VideoCompressor>>().then([this] (const Image& image) {

            // Split our image into components
            uint8_t* rPtr  = rEncoder.inputPicture->img.plane[0];
            uint8_t* g1Ptr = g1Encoder.inputPicture->img.plane[0];
            uint8_t* g2Ptr = g2Encoder.inputPicture->img.plane[0];
            uint8_t* bPtr  = bEncoder.inputPicture->img.plane[0];

            // Split out each spatial component from the bayer image
            for (int x = 0; x < image.payload.rows(); x += 2) {

                // Strip our rg row
                auto rg = image.payload.row(x);
                for (int y = 0; y < rg.cols(); y += 2) {
                    *(rPtr++)  = rg[y];
                    *(g1Ptr++) = rg[y + 1];
                }

                // Strip our gb row
                auto gb = image.payload.row(x + 1);
                for (int y = 0; y < gb.cols(); y += 2) {
                    *(g2Ptr++)  = gb[y];
                    *(bPtr++) = gb[y + 1];
                }
            }

            auto msg = std::make_unique<CompressedImage>();
            msg->timestamp = image.timestamp;

            int iNals;
            int size;

            // Scale R to YUV420 and encode
            size = x264_encoder_encode(rEncoder.encoder.get(), &rEncoder.nals, &iNals, rEncoder.inputPicture.get(), &rEncoder.outputPicture);
            msg->payloads.emplace_back(rEncoder.nals->p_payload, rEncoder.nals->p_payload + size);

            // Scale G1 to YUV420 and encode
            size = x264_encoder_encode(g1Encoder.encoder.get(), &g1Encoder.nals, &iNals, g1Encoder.inputPicture.get(), &g1Encoder.outputPicture);
            msg->payloads.emplace_back(g1Encoder.nals->p_payload, g1Encoder.nals->p_payload + size);

            // Scale G2 to YUV420 and encode
            size = x264_encoder_encode(g2Encoder.encoder.get(), &g2Encoder.nals, &iNals, g2Encoder.inputPicture.get(), &g2Encoder.outputPicture);
            msg->payloads.emplace_back(g2Encoder.nals->p_payload, g2Encoder.nals->p_payload + size);

            // Scale B to YUV420 and encode
            size = x264_encoder_encode(bEncoder.encoder.get(), &bEncoder.nals, &iNals, bEncoder.inputPicture.get(), &bEncoder.outputPicture);
            msg->payloads.emplace_back(bEncoder.nals->p_payload, bEncoder.nals->p_payload + size);

            emit(msg);
        });
    }
}
}
}
