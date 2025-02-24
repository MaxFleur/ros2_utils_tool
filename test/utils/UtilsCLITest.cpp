#include "catch_ros2/catch_ros2.hpp"

#include "UtilsCLI.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_cpp/writer.hpp"

#include "sensor_msgs/msg/image.hpp"

#include <filesystem>

TEST_CASE("Utils CLI Testing", "[utils]") {
    QStringList arguments { "argument1", "arg2", "test_3" };

    SECTION("Contains invalid test") {
        arguments.append("-h");
        REQUIRE(Utils::CLI::containsInvalidArguments(arguments, { "-h", "--help" }) == false);

        arguments.pop_back();
        arguments.append("--help");
        REQUIRE(Utils::CLI::containsInvalidArguments(arguments, { "-h", "--help" }) == false);

        arguments.append("-t");
        REQUIRE(Utils::CLI::containsInvalidArguments(arguments, { "-h", "--help" }) == true);

        arguments.pop_back();
        arguments.append("--test");
        REQUIRE(Utils::CLI::containsInvalidArguments(arguments, { "-h", "--help" }) == true);
    }
    SECTION("Contains test") {
        REQUIRE(Utils::CLI::containsArguments(arguments, "-t", "--test") == false);

        arguments.append("-t");
        REQUIRE(Utils::CLI::containsArguments(arguments, "-t", "--test") == true);

        arguments.pop_back();
        arguments.append("--test");
        REQUIRE(Utils::CLI::containsArguments(arguments, "-t", "--test") == true);
    }
    SECTION("Argument index test") {
        REQUIRE(Utils::CLI::getArgumentsIndex(arguments, "-t", "--test") == -1);

        arguments.append("-t");
        REQUIRE(Utils::CLI::getArgumentsIndex(arguments, "-t", "--test") == 3);

        arguments.pop_back();
        arguments.insert(1, "--test");
        REQUIRE(Utils::CLI::getArgumentsIndex(arguments, "-t", "--test") == 1);

        arguments.removeAt(1);
        arguments.push_front("--test");
        arguments.insert(1, "--t");
        arguments.insert(2, "--test");
        REQUIRE(Utils::CLI::getArgumentsIndex(arguments, "-t", "--test") == 2);
    }
    SECTION("Argument validity test") {
        auto parameter = 42;

        REQUIRE(Utils::CLI::checkArgumentValidity(arguments, "-t", "--test", parameter, 1, 100) == true);

        arguments.append("-t");
        REQUIRE(Utils::CLI::checkArgumentValidity(arguments, "-t", "--test", parameter, 1, 100) == false);

        arguments.append("42");
        REQUIRE(Utils::CLI::checkArgumentValidity(arguments, "-t", "--test", parameter, 1, 10) == false);
        REQUIRE(Utils::CLI::checkArgumentValidity(arguments, "-t", "--test", parameter, 50, 100) == false);
        REQUIRE(Utils::CLI::checkArgumentValidity(arguments, "-t", "--test", parameter, 1, 100) == true);
    }
    SECTION("Progress string test") {
        auto progressString = Utils::CLI::drawProgressString(0);
        REQUIRE(progressString == "--------------------------------------------------");
        progressString = Utils::CLI::drawProgressString(10);
        REQUIRE(progressString == "#####---------------------------------------------");
        progressString = Utils::CLI::drawProgressString(25);
        REQUIRE(progressString == "############--------------------------------------");
        progressString = Utils::CLI::drawProgressString(100);
        REQUIRE(progressString == "##################################################");
    }
    SECTION("Topic name valid position test") {
        arguments.append("-t");
        REQUIRE(Utils::CLI::isTopicParameterAtValidPosition(arguments) == false);
        arguments.append("/topic_name");
        REQUIRE(Utils::CLI::isTopicParameterAtValidPosition(arguments) == true);
    }
    SECTION("Topic name valid test") {
        const auto bagDirectory = std::filesystem::path("test_bag_file");
        std::filesystem::remove_all(bagDirectory);

        rosbag2_cpp::Writer writer;
        writer.open(bagDirectory);

        for (auto i = 0; i < 5; i++) {
            sensor_msgs::msg::Image imageMessage;
            imageMessage.width = 1;
            imageMessage.height = 1;
            writer.write(imageMessage, "/topic_image", rclcpp::Clock().now());
        }
        writer.close();

        QString topicName = "";
        REQUIRE(Utils::CLI::isTopicNameValid(arguments, "test_bag_file", "sensor_msgs/msg/Image", topicName) == true);
        arguments.append("-t");
        REQUIRE(Utils::CLI::isTopicNameValid(arguments, "test_bag_file", "sensor_msgs/msg/Image", topicName) == false);
        arguments.append("/random_topic");
        REQUIRE(Utils::CLI::isTopicNameValid(arguments, "test_bag_file", "sensor_msgs/msg/Image", topicName) == false);
        arguments.pop_back();
        arguments.append("/topic_image");
        REQUIRE(Utils::CLI::isTopicNameValid(arguments, "test_bag_file", "sensor_msgs/msg/PointCloud2", topicName) == false);
        REQUIRE(Utils::CLI::isTopicNameValid(arguments, "test_bag_file", "sensor_msgs/msg/Image", topicName) == true);

        std::filesystem::remove_all(bagDirectory);
    }
}
