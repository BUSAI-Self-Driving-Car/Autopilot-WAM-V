#ifndef UTILITY_CONVERT_YAML_OPENCV_HPP
#define UTILITY_CONVERT_YAML_OPENCV_HPP

#include <yaml-cpp/yaml.h>
#include <opencv2/core/core.hpp>

namespace YAML {

    template<>
    struct convert<cv::Point3f> {
        static Node encode(const cv::Point3f& rhs) {
            Node node;
            node.push_back(rhs.x);
            node.push_back(rhs.y);
            node.push_back(rhs.z);
            return node;
        }

        static bool decode(const Node& node, cv::Point3f& rhs) {
            if(!node.IsSequence() || node.size() != 3) {
                return false;
            }
            rhs.x = node[0].as<float>();
            rhs.y = node[1].as<float>();
            rhs.z = node[2].as<float>();
            return true;
        }
    };

    template<>
    struct convert<cv::Size> {
        static Node encode(const cv::Size& rhs) {
            Node node;
            node.push_back(rhs.width);
            node.push_back(rhs.height);
            return node;
        }

        static bool decode(const Node& node, cv::Size& rhs) {
            if(!node.IsSequence() || node.size() != 2) {
                return false;
            }
            rhs.width = node[0].as<int>();
            rhs.height = node[1].as<int>();
            return true;
        }
    };

}

#endif // UTILITY_CONVERT_YAML_OPENCV_HPP
