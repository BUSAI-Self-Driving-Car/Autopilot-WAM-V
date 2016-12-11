#include "VideoCompressor.h"

#include "extension/Configuration.h"

namespace module {
namespace support {
namespace logging {

    using extension::Configuration;

    VideoCompressor::VideoCompressor(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("VideoCompressor.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file VideoCompressor.yaml

            int width  = config["width"];
            int height = config["height"];

            // Setup our settings
            x264_param_t param;
            x264_param_default_preset(&param, "ultrafast", "zerolatency");
            param.i_threads = X264_THREADS_AUTO;
            param.i_width = width / 2;
            param.i_height = height / 2;
            param.i_fps_num = 30;
            param.i_fps_den = 1;
            // Intra refres:
            param.i_keyint_max = 30;
            param.b_intra_refresh = 1;
            //Rate control:
            param.rc.i_rc_method = X264_RC_CRF;
            param.rc.f_rf_constant = 25;
            param.rc.f_rf_constant_max = 35;

            //For streaming:
            param.b_repeat_headers = 1;
            param.b_annexb = 1;
            x264_param_apply_profile(&param, "baseline");

            swsContext = sws_getContext(width / 2, height / 2, AV_PIX_FMT_GRAY8, width / 2, height / 2, AV_PIX_FMT_YUV420P, flags, nullptr, nullptr, nullptr);
        });

        on<Trigger<Image>>().then([this] (const Image& image) {

            using cmat = Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>;

            // Split our image into components
            int data_size = (image.dimension[0] * image.dimension[0]) / 4;

            // Allocate some memory for each of the bytes
            std::vector<char> r;
            r.reserve(data_size);
            std::vector g1;
            g1.reserve(data_size);
            std::vector g2;
            g2.reserve(data_size);
            std::vector g3;
            b.reserve(data_size);

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



            // // Scale R
            // sws_scale(swsContext.get(),  r.data(), &stride, 0, param.i_height, pic_in.img.plane, pic_in.img.i_stride);

            // // Scale G1
            // sws_scale(swsContext.get(), g1.data(), &stride, 0, param.i_height, pic_in.img.plane, pic_in.img.i_stride);

            // // Scale G2
            // sws_scale(swsContext.get(), g2.data(), &stride, 0, param.i_height, pic_in.img.plane, pic_in.img.i_stride);

            // // Scale B
            // sws_scale(swsContext.get(),  b.data(), &stride, 0, param.i_height, pic_in.img.plane, pic_in.img.i_stride);








            // For now we just assume it's BGGR bayer image

            // Red image is odd cols, odd pixels

            // Blue image is even cols, even pixels



            // Compress



            // Emit

            //

        });
    }
}
}
}
