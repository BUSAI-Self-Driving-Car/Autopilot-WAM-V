#ifndef MODULE_INPUT_SPINNAKERCAMERA_H
#define MODULE_INPUT_SPINNAKERCAMERA_H

#include <nuclear>
#include <string>
#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

#include "message/vision/Image.h"

#include "utility/vision/fourcc.h"

namespace module {
namespace input {

    struct ImageEvent : public Spinnaker::ImageEvent {
        ImageEvent(const std::string& name, const std::string& serialNumber, Spinnaker::CameraPtr&& camera, NUClear::Reactor& reactor, const utility::vision::FOURCC& fourcc)
            : name(name), serialNumber(serialNumber), camera(std::move(camera)), reactor(reactor), fourcc(fourcc) {}
        ~ImageEvent()
        {
            if (camera)
            {
                camera->EndAcquisition();
                camera->UnregisterEvent(*this);
                camera->DeInit();
            }
        }

        std::string name;
        std::string serialNumber;
        Spinnaker::CameraPtr camera;
        NUClear::Reactor& reactor;
        utility::vision::FOURCC fourcc;

        void OnImageEvent(Spinnaker::ImagePtr image) {

            // Check image retrieval status
            if (!image->IsIncomplete())
            {
                auto msg = std::make_unique<message::vision::Image>(NUClear::clock::time_point(std::chrono::nanoseconds(image->GetTimeStamp())));

                msg->camera_id = serialNumber;
                msg->format = fourcc;
                msg->dimensions[0] = image->GetWidth();
                msg->dimensions[1] = image->GetHeight();

                msg->payload = Eigen::Map<Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(reinterpret_cast<uint8_t*>(image->GetData()), image->GetHeight(), image->GetStride());

                reactor.emit(msg);
            }
        }
    };

    class SpinnakerCamera : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the SpinnakerCamera reactor.
        explicit SpinnakerCamera(std::unique_ptr<NUClear::Environment> environment);

    private:
        Spinnaker::SystemPtr system;
        Spinnaker::CameraList camList;
        std::map<std::string, std::unique_ptr<ImageEvent>> cameras;
    };

}
}

#endif  // MODULE_INPUT_SPINNAKERCAMERA_H
