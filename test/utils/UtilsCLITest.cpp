#include "catch_ros2/catch_ros2.hpp"

#include "UtilsCLI.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_cpp/writer.hpp"

#include "sensor_msgs/msg/image.hpp"

#include <filesystem>

TEST_CASE("Utils CLI Testing", "[utils]") {
    QStringList arguments { "argument1", "arg2", "test_3" };

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

    SECTION("Contains invalid parameters test") {
        arguments.append("-h");
        REQUIRE(Utils::CLI::containsInvalidParameters(arguments, { "-h", "--help" }) == std::nullopt);

        arguments.pop_back();
        arguments.append("--help");
        REQUIRE(Utils::CLI::containsInvalidParameters(arguments, { "-h", "--help" }) == std::nullopt);

        arguments.append("-t");
        REQUIRE(Utils::CLI::containsInvalidParameters(arguments, { "-h", "--help" }) == "-t");

        arguments.pop_back();
        arguments.append("--test");
        REQUIRE(Utils::CLI::containsInvalidParameters(arguments, { "-h", "--help" }) == "--test");
    }
    SECTION("Contains arguments test") {
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
    SECTION("Check argument validity test") {
        auto parameter = 42;

        REQUIRE(Utils::CLI::checkArgumentValidity(arguments, "-t", "--test", parameter, 1, 100) == true);

        arguments.append("-t");
        REQUIRE(Utils::CLI::checkArgumentValidity(arguments, "-t", "--test", parameter, 1, 100) == false);

        arguments.append("42");
        REQUIRE(Utils::CLI::checkArgumentValidity(arguments, "-t", "--test", parameter, 1, 10) == false);
        REQUIRE(Utils::CLI::checkArgumentValidity(arguments, "-t", "--test", parameter, 50, 100) == false);
        REQUIRE(Utils::CLI::checkArgumentValidity(arguments, "-t", "--test", parameter, 1, 100) == true);
    }
    SECTION("Topic parameter position test") {
        arguments.append("-t");
        REQUIRE_THROWS_WITH(Utils::CLI::checkTopicParameterPosition(arguments), "Please enter a valid topic name!");
        arguments.append("/topic_name");
        REQUIRE_NOTHROW(Utils::CLI::checkTopicParameterPosition(arguments));
    }
    SECTION("Topic name validity test") {
        QString topicName = "";
        REQUIRE_NOTHROW(Utils::CLI::checkTopicNameValidity(arguments, "test_bag_file", "sensor_msgs/msg/Image", topicName));
        arguments.append("-t");
        CHECK_THROWS_WITH(Utils::CLI::checkTopicNameValidity(arguments, "test_bag_file", "sensor_msgs/msg/Image", topicName),
                          "Please enter a valid topic name!");
        arguments.append("/random_topic");
        CHECK_THROWS_WITH(Utils::CLI::checkTopicNameValidity(arguments, "test_bag_file", "sensor_msgs/msg/Image", topicName),
                          "Topic '/random_topic' has not been found in the bag file!");
        arguments.pop_back();
        arguments.append("/topic_image");
        CHECK_THROWS_WITH(Utils::CLI::checkTopicNameValidity(arguments, "test_bag_file", "sensor_msgs/msg/PointCloud2", topicName),
                          "Topic '/topic_image' doesn't have the correct type!");
        REQUIRE_NOTHROW(Utils::CLI::checkTopicNameValidity(arguments, "test_bag_file", "sensor_msgs/msg/Image", topicName));

        std::filesystem::remove_all(bagDirectory);
    }
    SECTION("Check bag source directory test") {
        REQUIRE_NOTHROW(Utils::CLI::checkBagSourceDirectory("test_bag_file"));

        CHECK_THROWS_WITH(Utils::CLI::checkBagSourceDirectory("/random/location"), "Bag file not found. Make sure that the bag file exists!");
        std::filesystem::create_directory("invalid_bag_dir");
        CHECK_THROWS_WITH(Utils::CLI::checkBagSourceDirectory("invalid_bag_dir"), "The directory does not contain a bag file!");
        std::filesystem::remove("invalid_bag_dir");
    }
    SECTION("Check parent directory test") {
        CHECK_THROWS_WITH(Utils::CLI::checkParentDirectory("test_bag_file"), "Invalid target directory. Please enter a valid one!");
        CHECK_THROWS_WITH(Utils::CLI::checkParentDirectory("test_bag_file", false), "Invalid source directory. Please enter a valid one!");

        std::filesystem::create_directories("parent/target");
        REQUIRE_NOTHROW(Utils::CLI::checkParentDirectory("parent/target"));
        std::filesystem::remove_all("parent");
    }
    SECTION("Check target topic test") {
        QString topicName = "";
        REQUIRE_NOTHROW(Utils::CLI::checkForTargetTopic("test_bag_file", topicName, true));
        REQUIRE(topicName == "/topic_image");

        CHECK_THROWS_WITH(Utils::CLI::checkForTargetTopic("test_bag_file", topicName, false), "The bag file does not contain any point cloud topics!");
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

    std::filesystem::remove_all("test_bag_file");
}
