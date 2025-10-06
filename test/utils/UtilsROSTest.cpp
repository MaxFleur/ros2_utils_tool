#include "catch_ros2/catch_ros2.hpp"

#include "UtilsROS.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_transport/bag_rewrite.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

#include <filesystem>

TEST_CASE("Utils ROS Testing", "[utils]") {
    const auto bagDirectory = std::filesystem::path("test_bag_file");
    std::filesystem::remove_all(bagDirectory);

    auto writer = std::make_shared<rosbag2_cpp::Writer>();
    writer->open(bagDirectory);

    const auto qString = QString::fromStdString(bagDirectory);

    for (auto i = 0; i < 5; i++) {
        sensor_msgs::msg::Image imageMessage;
        imageMessage.width = 1;
        imageMessage.height = 1;
        writer->write(imageMessage, "/topic_image", rclcpp::Clock().now());
    }
    for (auto i = 0; i < 3; i++) {
        Utils::ROS::writeMessageToBag(std_msgs::msg::String(), "Message " + std::to_string(i), writer, "/topic_string", rclcpp::Clock().now());
    }
    for (auto i = 0; i < 10; i++) {
        Utils::ROS::writeMessageToBag(std_msgs::msg::Int32(), i, writer, "/topic_integer", rclcpp::Clock().now());
    }
    writer->close();

    // Create compressed bag file
    rosbag2_storage::StorageOptions inputStorage;
    inputStorage.uri = bagDirectory;

    rosbag2_storage::StorageOptions outputStorage;
    outputStorage.uri = "compressed_bag_file";

    rosbag2_transport::RecordOptions outputRecord;
#ifdef ROS_HUMBLE
    outputRecord.all = true;
#else
    outputRecord.all_topics = true;
#endif
    outputRecord.compression_format = "zstd";
    outputRecord.compression_mode = "file";
    outputRecord.compression_queue_size = 0;

    std::vector<std::pair<rosbag2_storage::StorageOptions, rosbag2_transport::RecordOptions> > outputBags;
    outputBags.push_back({ outputStorage, outputRecord });
    rosbag2_transport::bag_rewrite({ inputStorage }, outputBags);

    SECTION("Does dir contain bag file test") {
        auto contains = Utils::ROS::doesDirectoryContainBagFile("path/to/random/location");
        REQUIRE(contains == false);
        contains = Utils::ROS::doesDirectoryContainBagFile(qString);
        REQUIRE(contains == true);
    }
    SECTION("Does dir contain compressed bag file test") {
        auto contains = Utils::ROS::doesDirectoryContainCompressedBagFile("path/to/random/location");
        REQUIRE(contains == false);
        contains = Utils::ROS::doesDirectoryContainCompressedBagFile(qString);
        REQUIRE(contains == false);
        contains = Utils::ROS::doesDirectoryContainCompressedBagFile("compressed_bag_file");
        REQUIRE(contains == true);
    }
    SECTION("Topics and services test") {
        auto runThread = true;
        // Run some constant publishing in the background
        auto publishingThread = std::thread([&runThread] {
            auto node = std::make_shared<rclcpp::Node>("topics_publisher");
            auto publisher = node->create_publisher<std_msgs::msg::Int32>("/example", 10);
            rclcpp::Rate rate(50);

            while (runThread) {
                auto message = std_msgs::msg::Int32();
                message.data = 0;
                publisher->publish(message);
                rate.sleep();
            }
        });

        SECTION("Topics test") {
            const auto& topics = Utils::ROS::getTopicInformation();
            const auto it = std::find_if(topics.begin(), topics.end(), [] (const auto& element) {
                return element.first == "/example";
            });
            REQUIRE(it != topics.end());
            // Check for example topic
            REQUIRE(it->second[0] == "std_msgs/msg/Int32");
            REQUIRE(it->second[1] == "1");
            REQUIRE(it->second[2] == "0");
        }
        SECTION("Services test") {
            // Running the publishing node should create lots of services running in the background
            const auto& map = Utils::ROS::getServiceNamesAndTypes();
            REQUIRE(map.size() != 0);
        }
        runThread = false;
        publishingThread.join();
    }
    SECTION("Get bag metadata test") {
        const auto& metadata = Utils::ROS::getBagMetadata(qString);
        REQUIRE(metadata.message_count == 18);

        const auto& topicsWithInformation = metadata.topics_with_message_count;
        REQUIRE(topicsWithInformation.size() == 3);

        auto metaDataTopic = topicsWithInformation.at(0).topic_metadata;
        REQUIRE(metaDataTopic.name == "/topic_integer");
        REQUIRE(metaDataTopic.type == "std_msgs/msg/Int32");
        metaDataTopic = topicsWithInformation.at(1).topic_metadata;
        REQUIRE(metaDataTopic.name == "/topic_string");
        REQUIRE(metaDataTopic.type == "std_msgs/msg/String");
        metaDataTopic = topicsWithInformation.at(2).topic_metadata;
        REQUIRE(metaDataTopic.name == "/topic_image");
        REQUIRE(metaDataTopic.type == "sensor_msgs/msg/Image");
    }
    SECTION("Get topic test") {
        auto topic = Utils::ROS::getTopicInBag(qString, "/topic_image");
        REQUIRE(topic != std::nullopt);
        topic = Utils::ROS::getTopicInBag(qString, "/topic_string");
        REQUIRE(topic != std::nullopt);
        topic = Utils::ROS::getTopicInBag(qString, "/topic_should_not_be_included");
        REQUIRE(topic == std::nullopt);
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
        REQUIRE(messageCount == std::nullopt);
    }
    SECTION("Topic type test") {
        auto topicType = Utils::ROS::getTopicType(qString, "/topic_image");
        REQUIRE(topicType == "sensor_msgs/msg/Image");
        topicType = Utils::ROS::getTopicType(qString, "/topic_string");
        REQUIRE(topicType == "std_msgs/msg/String");
        topicType = Utils::ROS::getTopicType(qString, "/topic_should_not_be_included");
        REQUIRE(topicType == std::nullopt);
    }
    SECTION("First topic with type test") {
        auto topicName = Utils::ROS::getFirstTopicWithCertainType(qString, "sensor_msgs/msg/Image");
        REQUIRE(*topicName == "/topic_image");
        topicName = Utils::ROS::getFirstTopicWithCertainType(qString, "std_msgs/msg/String");
        REQUIRE(*topicName == "/topic_string");
        topicName = Utils::ROS::getFirstTopicWithCertainType(qString, "std_msgs/msg/Int32");
        REQUIRE(topicName == "/topic_integer");
        topicName = Utils::ROS::getFirstTopicWithCertainType(qString, "std_msgs/msg/Float32");
        REQUIRE(topicName == std::nullopt);
    }
    SECTION("Video topics test") {
        const auto videoTopics = Utils::ROS::getBagTopics(qString, "sensor_msgs/msg/Image");
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
    std::filesystem::remove_all("compressed_bag_file");
}
