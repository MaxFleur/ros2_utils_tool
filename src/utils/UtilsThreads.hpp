#pragma once

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

// Thread related util functions
namespace Utils::Threads
{
// Keep these header-only to prevent globalized cv_bridge imports for all tools using utils
inline cv::Mat
convertCompressedImageMessageToMat(rcutils_uint8_array_t&                                   serializedData,
                                   rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serialization,
                                   std::shared_ptr<sensor_msgs::msg::CompressedImage>       message)
{
    // Deserialize
    rclcpp::SerializedMessage serializedMessage(serializedData);
    serialization.deserialize_message(&serializedMessage, message.get());

    cv::Mat buffer(1, static_cast<int>(message->data.size()), CV_8UC1, message->data.data());
    return cv::imdecode(buffer, cv::IMREAD_COLOR);
}


inline cv::Mat
convertImageMessageToMat(rcutils_uint8_array_t&                         serializedData,
                         rclcpp::Serialization<sensor_msgs::msg::Image> serialization,
                         cv_bridge::CvImagePtr                          cvPointer,
                         std::shared_ptr<sensor_msgs::msg::Image>       message)
{
    // Deserialize
    rclcpp::SerializedMessage serializedMessage(serializedData);
    serialization.deserialize_message(&serializedMessage, message.get());

    cvPointer = cv_bridge::toCvCopy(message, message->encoding);
    return cvPointer->image;
}
}
