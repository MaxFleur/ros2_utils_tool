#include "catch_ros2/catch_ros2.hpp"

#include "BagPlayer.hpp"
#include "BagRecorder.hpp"
#include "DummyBagThread.hpp"
#include "Parameters.hpp"
#include "UtilsROS.hpp"

#include <cv_bridge/cv_bridge.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

#include <filesystem>

TEST_CASE("Bah Handlers Testing", "[threads]") {
    SECTION("Bag Player Test") {
        rclcpp::Rate rate(10);

        Parameters::DummyBagParameters dummyBagParameters;
        dummyBagParameters.sourceDirectory = "./test_bag";
        dummyBagParameters.messageCount = 100;
        dummyBagParameters.topics.push_back({ { "/image" }, "Image" });
        dummyBagParameters.topics.push_back({ { "/integer" }, "Integer" });
        dummyBagParameters.topics.push_back({ { "/string" }, "String" });

        auto* const thread = new DummyBagThread(dummyBagParameters, std::thread::hardware_concurrency());
        QObject::connect(thread, &DummyBagThread::finished, thread, &QObject::deleteLater);

        thread->start();
        thread->wait();

        Parameters::PlayBagParameters playBagParameters;
        playBagParameters.sourceDirectory = "./test_bag";
        playBagParameters.topics.push_back({ { "/image" }, false });
        playBagParameters.topics.push_back({ { "/integer" }, true });
        playBagParameters.topics.push_back({ { "/string" }, true });
        playBagParameters.rate = 1.0;
        playBagParameters.loop = true;

        std::vector<std::pair<std::string, std::array<std::string, 3> > > topicInformation;
        {
            // Ensure correct BagPlayer cleaning up
            auto bagPlayer = std::make_unique<BagPlayer>(playBagParameters);
            rate.sleep();
            topicInformation = Utils::ROS::getTopicInformation();
        }

        const auto containsKey = [&topicInformation] (const std::string& key) {
            auto it = std::ranges::find_if(topicInformation, [&key] (const auto& topic) {
                return topic.first == key;
            });

            return it != topicInformation.end();
        };

        REQUIRE(containsKey("/integer"));
        REQUIRE(containsKey("/string"));
        REQUIRE(!containsKey("/image"));

        std::filesystem::remove_all("./test_bag");
    }

    SECTION("Bag Recorder Test") {
        Parameters::RecordBagParameters recordBagParameters;
        recordBagParameters.sourceDirectory = "./recorded_bag";

        rclcpp::Rate rate(20);
        auto shouldDelete = false;

        const auto sendMessages = [&rate] {
            auto node = std::make_shared<rclcpp::Node>("tests_publisher");
            auto publisherInteger = node->create_publisher<std_msgs::msg::Int32>("/integer", 10);
            auto publisherString = node->create_publisher<std_msgs::msg::String>("/string", 10);
            auto publisherImage = node->create_publisher<sensor_msgs::msg::Image>("/image", 10);
            cv::Mat mat(100, 100, CV_8UC3, cv::Scalar(255, 0, 0));

            // Send some messages with smaller intervals between
            for (auto i = 0; i < 10; i++) {
                auto messageInteger = std_msgs::msg::Int32();
                messageInteger.data = i;
                auto messageString = std_msgs::msg::String();
                messageString.data = "Message " + std::to_string(i);

                auto messageImage = sensor_msgs::msg::Image();
                std_msgs::msg::Header header;
                header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
                const auto cvBridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, mat);
                cvBridge.toImageMsg(messageImage);

                publisherInteger->publish(messageInteger);
                publisherString->publish(messageString);
                publisherImage->publish(messageImage);
                rate.sleep();
            }
        };

        SECTION("General unspecified run") {
            {
                auto bagRecorder = std::make_unique<BagRecorder>(recordBagParameters);
                rate.sleep();
                sendMessages();
                rate.sleep();
            }
            const auto& metaData = Utils::ROS::getBagMetadata("./recorded_bag");
            REQUIRE(metaData.topics_with_message_count.size() == 0);
        }
        SECTION("Specified topic run") {
            SECTION("Uncompressed") {
                const auto containsTopic = [] (const std::vector<rosbag2_storage::TopicInformation>& topicInformation, const std::string& key) {
                    auto it = std::ranges::find_if(topicInformation, [&key] (const auto& topic) {
                        return topic.topic_metadata.name == key;
                    });

                    return it != topicInformation.end();
                };

                recordBagParameters.includeUnpublishedTopics = false;
                recordBagParameters.topics.push_back({ { "/image" }, false });
                recordBagParameters.topics.push_back({ { "/integer" }, true });
                recordBagParameters.topics.push_back({ { "/string" }, true });

                {
                    auto bagRecorder = std::make_unique<BagRecorder>(recordBagParameters);
                    rate.sleep();
                    sendMessages();
                    rate.sleep();
                }

                const auto& metaData = Utils::ROS::getBagMetadata("./recorded_bag");
                const auto& topics = metaData.topics_with_message_count;
                REQUIRE(topics.size() == 2);
                REQUIRE(containsTopic(topics, "/integer"));
                REQUIRE(containsTopic(topics, "/string"));
                REQUIRE(!containsTopic(topics, "/image"));
            }
            SECTION("Compressed") {
                recordBagParameters.topics.push_back({ { "/image" }, false });
                recordBagParameters.topics.push_back({ { "/integer" }, true });
                recordBagParameters.topics.push_back({ { "/string" }, true });
                recordBagParameters.useCompression = true;
                recordBagParameters.isCompressionFile = true;

                {
                    auto bagRecorder = std::make_unique<BagRecorder>(recordBagParameters);
                    rate.sleep();
                    sendMessages();
                    rate.sleep();
                }

                REQUIRE(Utils::ROS::doesDirectoryContainCompressedBagFile("./recorded_bag") == true);
                shouldDelete = true;
            }
        }

        // This will be executed before EACH segment, so set true at the very end
        if (shouldDelete) {
            std::filesystem::remove_all("./recorded_bag");
        }
    }
}
