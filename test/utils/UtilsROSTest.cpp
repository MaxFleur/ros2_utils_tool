#include "catch_ros2/catch_ros2.hpp"

#include "UtilsROS.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_cpp/writer.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

#include <filesystem>

TEST_CASE("Utils ROS Testing", "[utils]") {
    const auto bagDirectory = std::filesystem::path("test_bag_file");
    std::filesystem::remove_all(bagDirectory);

    rosbag2_cpp::Writer writer;
    writer.open(bagDirectory);

    const auto qString = QString::fromStdString(bagDirectory);

    for (auto i = 0; i < 5; i++) {
        sensor_msgs::msg::Image imageMessage;
        imageMessage.width = 1;
        imageMessage.height = 1;
        writer.write(imageMessage, "/topic_image", rclcpp::Clock().now());
    }
    for (auto i = 0; i < 3; i++) {
        std_msgs::msg::String stringMessage;
        stringMessage.data = "example string";
        writer.write(stringMessage, "/topic_string", rclcpp::Clock().now());
    }
    writer.close();

    SECTION("Does dir contain bag file test") {
        auto contains = Utils::ROS::doesDirectoryContainBagFile("path/to/random/location");
        REQUIRE(contains == false);
        contains = Utils::ROS::doesDirectoryContainBagFile(qString);
        REQUIRE(contains == true);
    }
    SECTION("Contains topic name test") {
        auto contains = Utils::ROS::doesBagContainTopicName(qString, "/topic_image");
        REQUIRE(contains == true);
        contains = Utils::ROS::doesBagContainTopicName(qString, "/topic_string");
        REQUIRE(contains == true);
        contains = Utils::ROS::doesBagContainTopicName(qString, "/topic_should_not_be_included");
        REQUIRE(contains == false);
    }
    SECTION("Topic message count test") {
        auto messageCount = Utils::ROS::getTopicMessageCount(qString, "/topic_image");
        REQUIRE(messageCount == 5);
        messageCount = Utils::ROS::getTopicMessageCount(qString, "/topic_string");
        REQUIRE(messageCount == 3);
        messageCount = Utils::ROS::getTopicMessageCount(qString, "/topic_should_not_be_included");
        REQUIRE(messageCount == 0);
    }
    SECTION("Topic type test") {
        auto topicType = Utils::ROS::getTopicType(qString, "/topic_image");
        REQUIRE(topicType == "sensor_msgs/msg/Image");
        topicType = Utils::ROS::getTopicType(qString, "/topic_string");
        REQUIRE(topicType == "std_msgs/msg/String");
        topicType = Utils::ROS::getTopicType(qString, "/topic_should_not_be_included");
        REQUIRE(topicType == "");
    }
    SECTION("First topic with type test") {
        auto topicName = Utils::ROS::getFirstTopicWithCertainType(qString, "sensor_msgs/msg/Image");
        REQUIRE(*topicName == "/topic_image");
        topicName = Utils::ROS::getFirstTopicWithCertainType(qString, "std_msgs/msg/String");
        REQUIRE(*topicName == "/topic_string");
        topicName = Utils::ROS::getFirstTopicWithCertainType(qString, "std_msgs/msg/Int");
        REQUIRE(topicName == std::nullopt);
    }
    SECTION("Video topics test") {
        const auto videoTopics = Utils::ROS::getBagVideoTopics(qString);
        REQUIRE(videoTopics.size() == 1);
        REQUIRE(videoTopics.at(0) == "/topic_image");
    }
    SECTION("Name ROS2 convention tests") {
        SECTION("Fails for special characters") {
            const auto followsConvention = Utils::ROS::isNameROS2Conform("!\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}");
            REQUIRE(followsConvention == false);
        }
        SECTION("Slash and underbrackets") {
            auto followsConvention = Utils::ROS::isNameROS2Conform("test_topic/");
            REQUIRE(followsConvention == false);
            followsConvention = Utils::ROS::isNameROS2Conform("test__topic");
            REQUIRE(followsConvention == false);
            followsConvention = Utils::ROS::isNameROS2Conform("//test_topic");
            REQUIRE(followsConvention == false);
        }
        SECTION("First char number") {
            auto followsConvention = Utils::ROS::isNameROS2Conform("0test_topic");
            REQUIRE(followsConvention == false);
            followsConvention = Utils::ROS::isNameROS2Conform("test_topic");
            REQUIRE(followsConvention == true);
        }
        SECTION("Tilde and slash") {
            auto followsConvention = Utils::ROS::isNameROS2Conform("test~topic");
            REQUIRE(followsConvention == false);
            followsConvention = Utils::ROS::isNameROS2Conform("test~/topic");
            REQUIRE(followsConvention == true);
        }
        SECTION("Balanced curly braces") {
            auto followsConvention = Utils::ROS::isNameROS2Conform("test{topic}");
            REQUIRE(followsConvention == true);
            followsConvention = Utils::ROS::isNameROS2Conform("test_{topic");
            REQUIRE(followsConvention == false);
            followsConvention = Utils::ROS::isNameROS2Conform("test_topic}");
            REQUIRE(followsConvention == false);
            followsConvention = Utils::ROS::isNameROS2Conform("test_{t{opic}");
            REQUIRE(followsConvention == false);
        }
    }

    std::filesystem::remove_all(bagDirectory);
}
