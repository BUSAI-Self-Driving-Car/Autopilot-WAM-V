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

            int width  = config["width"];
            int height = config["height"];

            // Setup our settings
            x264_param_default_preset(&compressionParams, "ultrafast", "zerolatency");
            compressionParams.i_threads = X264_THREADS_AUTO;
            compressionParams.i_width = width / 2;
            compressionParams.i_height = height / 2;
            compressionParams.i_fps_num = 30;
            compressionParams.i_fps_den = 1;
            // Intra refresh:
            compressionParams.i_keyint_max = 30;
            compressionParams.b_intra_refresh = 1;
            //Rate control:
            compressionParams.rc.i_rc_method = X264_RC_CRF;
            compressionParams.rc.f_rf_constant = 25;
            compressionParams.rc.f_rf_constant_max = 35;

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

            // Allocate pictures to swscale into
            std::function<void(x264_picture_t*)> picture_delete = [] (x264_picture_t* ptr) {
                x264_picture_clean(ptr);
                delete ptr;
            };
            rEncoder.inputPicture = std::unique_ptr<x264_picture_t, std::function<void(x264_picture_t*)>>(new x264_picture_t(), picture_delete);
            g1Encoder.inputPicture = std::unique_ptr<x264_picture_t, std::function<void(x264_picture_t*)>>(new x264_picture_t(), picture_delete);
            g2Encoder.inputPicture = std::unique_ptr<x264_picture_t, std::function<void(x264_picture_t*)>>(new x264_picture_t(), picture_delete);
            bEncoder.inputPicture = std::unique_ptr<x264_picture_t, std::function<void(x264_picture_t*)>>(new x264_picture_t(), picture_delete);

            x264_picture_alloc(rEncoder.inputPicture.get(), X264_CSP_I420, width, height);
            x264_picture_alloc(g1Encoder.inputPicture.get(), X264_CSP_I420, width, height);
            x264_picture_alloc(g2Encoder.inputPicture.get(), X264_CSP_I420, width, height);
            x264_picture_alloc(bEncoder.inputPicture.get(), X264_CSP_I420, width, height);

            swsContext = std::unique_ptr<SwsContext, std::function<void(SwsContext*)>>(
                sws_getContext(
                    width / 2
                  , height / 2
                  , AV_PIX_FMT_GRAY8
                  , width / 2
                  , height / 2
                  , AV_PIX_FMT_YUV420P
                  , SWS_FAST_BILINEAR | SWS_PRINT_INFO
                  , nullptr, nullptr, nullptr)
                , sws_freeContext);
        });

        on<Trigger<Image>, Buffer<4>, Sync<VideoCompressor>>().then([this] (const Image& image) {

            using cmat = Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>;

            // Split our image into components
            int data_size = (image.dimensions[0] * image.dimensions[0]) / 4;

            // Allocate some memory for each of the bytes
            std::vector<uint8_t> r;
            r.reserve(data_size);
            std::vector<uint8_t> g1;
            g1.reserve(data_size);
            std::vector<uint8_t> g2;
            g2.reserve(data_size);
            std::vector<uint8_t> b;
            b.reserve(data_size);

            // Split out each spatial component from the bayer image
            for (int x = 0; x < image.payload.cols(); x += 2) {

                // Strip our rg row and put it in
                auto rg = image.payload.col(x);
                for (int y = 0; rg.rows(); y += 2) {
                    r.push_back(rg[y]);
                    g1.push_back(rg[y + 1]);
                }

                // Strip our gb row
                auto gb = image.payload.col(x + 1);
                for (int y = 0; y < gb.rows(); y += 2) {
                    g2.push_back(gb[y]);
                    b.push_back(gb[y + 1]);
                }
            }

            auto msg = std::make_unique<CompressedImage>();
            msg->timestamp = image.timestamp;

            int iNals;
            int stride = 480;

            // Scale R to YUV420 and encode
            uint8_t* rd = r.data();
            sws_scale(swsContext.get(), &rd, &stride, 0, compressionParams.i_height, rEncoder.inputPicture->img.plane, rEncoder.inputPicture->img.i_stride);
            int rSize = x264_encoder_encode(rEncoder.encoder.get(), &rEncoder.nals, &iNals, rEncoder.inputPicture.get(), &rEncoder.outputPicture);
            msg->payloads.emplace_back(rEncoder.nals->p_payload, rEncoder.nals->p_payload + rSize);

            // Scale G1 to YUV420 and encode
            uint8_t* g1d = g1.data();
            sws_scale(swsContext.get(), &g1d, &stride, 0, compressionParams.i_height, g1Encoder.inputPicture->img.plane, g1Encoder.inputPicture->img.i_stride);
            int g1Size = x264_encoder_encode(g1Encoder.encoder.get(), &g1Encoder.nals, &iNals, g1Encoder.inputPicture.get(), &g1Encoder.outputPicture);
            msg->payloads.emplace_back(g1Encoder.nals->p_payload, g1Encoder.nals->p_payload + rSize);

            // Scale G2 to YUV420 and encode
            uint8_t* g2d = g2.data();
            sws_scale(swsContext.get(), &g2d, &stride, 0, compressionParams.i_height, g2Encoder.inputPicture->img.plane, g2Encoder.inputPicture->img.i_stride);
            int g2Size = x264_encoder_encode(g2Encoder.encoder.get(), &g2Encoder.nals, &iNals, g2Encoder.inputPicture.get(), &g2Encoder.outputPicture);
            msg->payloads.emplace_back(g2Encoder.nals->p_payload, g2Encoder.nals->p_payload + rSize);

            // Scale B to YUV420 and encode
            uint8_t* bd = b.data();
            sws_scale(swsContext.get(), &bd, &stride, 0, compressionParams.i_height, bEncoder.inputPicture->img.plane, bEncoder.inputPicture->img.i_stride);
            int bSize = x264_encoder_encode(bEncoder.encoder.get(), &bEncoder.nals, &iNals, bEncoder.inputPicture.get(), &bEncoder.outputPicture);
            msg->payloads.emplace_back(bEncoder.nals->p_payload, bEncoder.nals->p_payload + rSize);

            emit(std::move(msg));
        });
    }
}
}
}
