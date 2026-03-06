#pragma once

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

template<typename T, typename U>
concept ImageMessageParameter = (std::same_as<T, sensor_msgs::msg::Image> && std::same_as<U, sensor_msgs::msg::Image>) ||
                                (std::same_as<T, sensor_msgs::msg::CompressedImage> && std::same_as<U, sensor_msgs::msg::CompressedImage>);

// Thread related util functions
namespace Utils::Threads
{
template<typename T, typename U>
requires ImageMessageParameter<T, U>
cv::Mat
convertImageMessageToMat(rcutils_uint8_array_t&         serializedData,
                         const rclcpp::Serialization<T> serialization,
                         cv_bridge::CvImagePtr          cvPointer,
                         std::shared_ptr<U>             message)
{
    // Deserialize
    rclcpp::SerializedMessage serializedMessage(serializedData);
    serialization.deserialize_message(&serializedMessage, message.get());

    if constexpr (std::is_same_v<T, sensor_msgs::msg::Image>) {
        // Convert message to cv
        cvPointer = cv_bridge::toCvCopy(message, message->encoding);
        return cvPointer->image;
    } else {
        return cv::imdecode(message->data, cv::IMREAD_COLOR);
    }
}
}
