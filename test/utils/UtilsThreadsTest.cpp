#include "catch_ros2/catch_ros2.hpp"

#include "UtilsThreads.hpp"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"

#include <filesystem>

TEST_CASE("Utils Threads Testing", "[utils]") {
    // Write bag with 10 normal and 10 compressed image messages
    auto writer = std::make_shared<rosbag2_cpp::Writer>();
    writer->open("threads_bag");

    std_msgs::msg::Header header;
    cv_bridge::CvImage cvBridge;
    cvBridge.encoding = sensor_msgs::image_encodings::RGB8;

    auto imageMessage = std::make_shared<sensor_msgs::msg::Image>();
    auto imageMessageCompressed = std::make_shared<sensor_msgs::msg::CompressedImage>();

    auto red = 255;
    auto blue = 0;

    for (auto i = 0; i < 10; ++i) {
        auto timeStamp = rclcpp::Clock(RCL_ROS_TIME).now();
        cv::Mat mat(720, 1280, CV_8UC3, cv::Scalar(red, 0, blue));

        header.stamp = timeStamp;
        cvBridge.header = header;
        cvBridge.image = mat;
        cvBridge.toImageMsg(*imageMessage);
        cvBridge.toCompressedImageMsg(*imageMessageCompressed);

        writer->write(*imageMessage, "/image", timeStamp);
        writer->write(*imageMessageCompressed, "/image_compressed", timeStamp);

        red -= 20;
        blue += 20;
    }
    writer->close();

    red = 255;
    blue = 0;

    // Reread the messages
    rosbag2_storage::SerializedBagMessageSharedPtr message;
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
    rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serializationCompressed;
    cv_bridge::CvImagePtr cvPointer;

    auto reader = std::make_unique<rosbag2_cpp::Reader>();
    reader->open("threads_bag");

    const auto isInRange = [] (int value, int low, int high) {
        return (value >= low) && (value <= high);
    };

    for (auto i = 0; i < 20; ++i) {
        message = reader->read_next();
        auto frame = message->topic_name == "/image_compressed" ? Utils::Threads::convertCompressedImageMessageToMat(*message->serialized_data, serializationCompressed, imageMessageCompressed)
                                                                : Utils::Threads::convertImageMessageToMat(*message->serialized_data, serialization, cvPointer, imageMessage);

        REQUIRE(frame.cols == 1280);
        REQUIRE(frame.rows == 720);

        const auto& color = frame.at<cv::Vec3b>(cv::Point(0, 0));

        REQUIRE(isInRange(color[1], 0, 2));
        if (message->topic_name == "/image_compressed") {
            // Red and blue values are switched when compressed images are used, for whatever reasons
            // Also, due to the compression, we cannot use exact values, they are in a certain range instead
            REQUIRE(isInRange(color[0], blue - 5, blue + 5));
            REQUIRE(isInRange(color[2], red - 5, red + 5));
        } else {
            REQUIRE(color[0] == red);
            REQUIRE(color[2] == blue);
        }

        if (i % 2 == 0) {
            continue;
        }
        red -= 20;
        blue += 20;
    }

    reader->close();
    std::filesystem::remove_all("threads_bag");
}
