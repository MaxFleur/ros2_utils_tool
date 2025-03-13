#include "catch_ros2/catch_ros2.hpp"

#include "BagToImagesThread.hpp"
#include "BagToPCDsThread.hpp"
#include "BagToVideoThread.hpp"
#include "DummyBagThread.hpp"
#include "EditBagThread.hpp"
#include "MergeBagsThread.hpp"
#include "PCDsToBagThread.hpp"
#include "PublishImagesThread.hpp"
#include "PublishVideoThread.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"
#include "VideoToBagThread.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <filesystem>

#ifdef ROS_JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

// Because of parallelized bag writing, the topic order in the output bag files might be different with each run
// But we can instead check the topic's index inside the bagfile and the information associated with it
int
getTopicIndex(const std::vector<rosbag2_storage::TopicInformation>& topics,
              const std::string&                                    topicName)
{
    const auto index = std::find_if(topics.begin(), topics.end(), [topicName] (const auto& topic) {
        return topic.topic_metadata.name == topicName;
    });

    return index == topics.end() ? -1 : index - topics.begin();
}


std::array<int, 2>
getDirFileCountWithExtensions(const std::string& dir, const std::string& extension)
{
    auto numberOfFiles = 0;
    auto numberOfExtension = 0;
    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        numberOfFiles++;
        if (entry.path().extension() == extension) {
            numberOfExtension++;
        }
    }

    return { numberOfFiles, numberOfExtension };
}


template<typename T>
concept MessageType = std::same_as<T, std_msgs::msg::Int32> || std::same_as<T, std_msgs::msg::String> ||
                      std::same_as<T, sensor_msgs::msg::PointCloud2>;

template<typename T>
requires MessageType<T>
void
verifyMessages(const std::string& bagDirectory, const std::string& topicName,
               rclcpp::Serialization<T>& serialization, int startValue)
{
    rosbag2_cpp::Reader reader;
    reader.open(bagDirectory);

    auto index = startValue;
    while (reader.has_next()) {
        auto msg = reader.read_next();

        if (msg->topic_name != topicName) {
            continue;
        }

        rclcpp::SerializedMessage serializedMessage(*msg->serialized_data);
        auto rosMsg = std::make_shared<T>();
        serialization.deserialize_message(&serializedMessage, rosMsg.get());

        if constexpr (std::is_same_v<T, std_msgs::msg::Int32>) {
            REQUIRE(rosMsg->data == index + 1);
        } else if constexpr (std::is_same_v<T, std_msgs::msg::String>) {
            REQUIRE(rosMsg->data == "Message " + std::to_string(index + 1));
        } else {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr fileCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr messageCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

            pcl::fromROSMsg(*rosMsg, *messageCloud);
            std::stringstream formatedIterationCount;
            formatedIterationCount << std::setw(3) << std::setfill('0') << index + 1;
            pcl::io::loadPCDFile<pcl::PointXYZRGB>("./pcds/" + formatedIterationCount.str() + ".pcd", *fileCloud);

            REQUIRE_THAT(fileCloud->at(0).x, Catch::Matchers::WithinAbs(messageCloud->at(0).x, 0.001));
            REQUIRE_THAT(fileCloud->at(0).y, Catch::Matchers::WithinAbs(messageCloud->at(0).y, 0.001));
            REQUIRE_THAT(fileCloud->at(0).z, Catch::Matchers::WithinAbs(messageCloud->at(0).z, 0.001));
            REQUIRE_THAT(fileCloud->at(0).r, Catch::Matchers::WithinAbs(messageCloud->at(0).r, 0.001));
            REQUIRE_THAT(fileCloud->at(0).g, Catch::Matchers::WithinAbs(messageCloud->at(0).g, 0.001));
            REQUIRE_THAT(fileCloud->at(0).b, Catch::Matchers::WithinAbs(messageCloud->at(0).b, 0.001));
        }

        index++;
    }
    reader.close();
}


TEST_CASE("Threads Testing", "[threads]") {
    auto shouldDelete = false;

    SECTION("Dummy Bag Thread Test") {
        Parameters::DummyBagParameters parameters;
        parameters.sourceDirectory = "./dummy_bag";
        parameters.messageCount = 200;
        parameters.topics.push_back({ "Image", "/dummy_image" });
        parameters.topics.push_back({ "Integer", "/dummy_integer" });
        parameters.topics.push_back({ "String", "/dummy_string" });
        parameters.topics.push_back({ "Point Cloud", "/dummy_points" });

        auto* const thread = new DummyBagThread(parameters, std::thread::hardware_concurrency());
        QObject::connect(thread, &DummyBagThread::finished, thread, &QObject::deleteLater);

        thread->start();
        while (!thread->isFinished()) {
        }

        const auto& metaData = Utils::ROS::getBagMetadata("./dummy_bag");
        REQUIRE(metaData.message_count == 800);
        const auto& topics = metaData.topics_with_message_count;
        REQUIRE(topics.size() == 4);
        REQUIRE(topics.at(0).message_count == 200);
        REQUIRE(topics.at(1).message_count == 200);
        REQUIRE(topics.at(2).message_count == 200);
        REQUIRE(topics.at(3).message_count == 200);

        auto topicIndex = getTopicIndex(topics, "/dummy_image");
        REQUIRE(topicIndex < 4);
        REQUIRE(topics.at(topicIndex).topic_metadata.type == "sensor_msgs/msg/Image");
        topicIndex = getTopicIndex(topics, "/dummy_string");
        REQUIRE(topicIndex < 4);
        REQUIRE(topics.at(topicIndex).topic_metadata.type == "std_msgs/msg/String");
        topicIndex = getTopicIndex(topics, "/dummy_integer");
        REQUIRE(topicIndex < 4);
        REQUIRE(topics.at(topicIndex).topic_metadata.type == "std_msgs/msg/Int32");
        topicIndex = getTopicIndex(topics, "/dummy_points");
        REQUIRE(topicIndex < 4);
        REQUIRE(topics.at(topicIndex).topic_metadata.type == "sensor_msgs/msg/PointCloud2");

        rclcpp::Serialization<std_msgs::msg::Int32> serializationInt;
        rclcpp::Serialization<std_msgs::msg::String> serializationString;

        verifyMessages("./dummy_bag", "/dummy_int", serializationInt, 0);
        verifyMessages("./dummy_bag", "/dummy_string", serializationString, 0);
    }
    // Create edited bag out of dummy bag
    SECTION("Edit Bag Thread Test") {
        Parameters::EditBagParameters parameters;
        parameters.sourceDirectory = "./dummy_bag";
        parameters.targetDirectory = "./edited_bag";
        parameters.updateTimestamps = true;
        parameters.topics.push_back({ "", "/dummy_image", 0, 100, true });
        parameters.topics.push_back({ "", "/dummy_integer", 0, 200, false });
        parameters.topics.push_back({ "/renamed_string", "/dummy_string", 50, 150, true });

        auto* const thread = new EditBagThread(parameters, std::thread::hardware_concurrency());
        QObject::connect(thread, &EditBagThread::finished, thread, &QObject::deleteLater);

        thread->start();
        while (!thread->isFinished()) {
        }

        const auto& metadata = Utils::ROS::getBagMetadata("./edited_bag");
        REQUIRE(metadata.message_count == 200);
        const auto& topics = metadata.topics_with_message_count;
        REQUIRE(topics.size() == 2);
        auto topicIndex = getTopicIndex(topics, "/dummy_image");
        REQUIRE(topicIndex < 2);
        REQUIRE(topics.at(topicIndex).topic_metadata.type == "sensor_msgs/msg/Image");
        REQUIRE(topics.at(topicIndex).message_count == 100);
        topicIndex = getTopicIndex(topics, "/renamed_string");
        REQUIRE(topicIndex < 2);
        REQUIRE(topics.at(topicIndex).topic_metadata.type == "std_msgs/msg/String");
        REQUIRE(topics.at(topicIndex).message_count == 100);

        rclcpp::Serialization<std_msgs::msg::String> serializationString;
        verifyMessages("./edited_bag", "/renamed_string", serializationString, 50);
    }
    // Merge edited and dummy bag
    SECTION("Merge Bags Thread Test") {
        Parameters::MergeBagsParameters parameters;
        parameters.sourceDirectory = "./dummy_bag";
        parameters.secondSourceDirectory = "./edited_bag";
        parameters.targetDirectory = "./merged_bag";

        auto* const thread = new MergeBagsThread(parameters, std::thread::hardware_concurrency());
        QObject::connect(thread, &MergeBagsThread::finished, thread, &QObject::deleteLater);

        SECTION("Unique Topics") {
            parameters.topics.push_back({ "/dummy_integer", "./dummy_bag", true });
            parameters.topics.push_back({ "/renamed_string", "./edited_bag", true });

            thread->start();
            while (!thread->isFinished()) {
            }

            const auto& metadata = Utils::ROS::getBagMetadata("./merged_bag");
            REQUIRE(metadata.message_count == 300);
            const auto& topics = metadata.topics_with_message_count;
            REQUIRE(topics.size() == 2);
            auto topicIndex = getTopicIndex(topics, "/dummy_integer");
            REQUIRE(topicIndex < 2);
            REQUIRE(topics.at(topicIndex).topic_metadata.type == "std_msgs/msg/Int32");
            REQUIRE(topics.at(topicIndex).message_count == 200);
            topicIndex = getTopicIndex(topics, "/renamed_string");
            REQUIRE(topicIndex < 2);
            REQUIRE(topics.at(topicIndex).topic_metadata.type == "std_msgs/msg/String");
            REQUIRE(topics.at(topicIndex).message_count == 100);

            rclcpp::Serialization<std_msgs::msg::Int32> serializationInt;
            rclcpp::Serialization<std_msgs::msg::String> serializationString;

            verifyMessages("./merged_bag", "/dummy_integer", serializationInt, 0);
            verifyMessages("./merged_bag", "/renamed_string", serializationString, 50);
        }
        // Assure that two topics of the same name, but from different bags will be merged into one topic
        SECTION("Merged Topics") {
            parameters.topics.clear();
            parameters.topics.push_back({ "/dummy_image", "./dummy_bag", true });
            parameters.topics.push_back({ "/dummy_image", "./edited_bag", true });
            parameters.topics.push_back({ "/renamed_string", "./edited_bag", true });

            thread->start();
            while (!thread->isFinished()) {
            }

            const auto& metadata = Utils::ROS::getBagMetadata("./merged_bag");
            REQUIRE(metadata.message_count == 400);
            const auto& topics = metadata.topics_with_message_count;
            REQUIRE(topics.size() == 2);
            auto topicIndex = getTopicIndex(topics, "/renamed_string");
            REQUIRE(topicIndex < 2);
            REQUIRE(topics.at(topicIndex).topic_metadata.type == "std_msgs/msg/String");
            REQUIRE(topics.at(topicIndex).message_count == 100);
            topicIndex = getTopicIndex(topics, "/dummy_image");
            REQUIRE(topicIndex < 2);
            REQUIRE(topics.at(topicIndex).topic_metadata.type == "sensor_msgs/msg/Image");
            REQUIRE(topics.at(topicIndex).message_count == 300);

            rclcpp::Serialization<std_msgs::msg::String> serializationString;
            verifyMessages("./merged_bag", "/renamed_string", serializationString, 50);
        }

        std::filesystem::remove_all("./merged_bag");
    }

    SECTION("Bag to Video Thread Test") {
        Parameters::BagToVideoParameters parameters;
        parameters.sourceDirectory = "./dummy_bag";
        parameters.targetDirectory = "./video.mp4";
        parameters.topicName = "/dummy_image";

        auto* const thread = new BagToVideoThread(parameters);
        QObject::connect(thread, &BagToVideoThread::finished, thread, &QObject::deleteLater);

        const auto performVideoCheck = [] (const std::string& fileExtension, int codec, int fps, int blueValue, int greenValue, int redValue) {
            auto videoCapture = cv::VideoCapture("./video" + fileExtension);
            REQUIRE(videoCapture.get(cv::CAP_PROP_FRAME_COUNT) == 200);
            REQUIRE(videoCapture.get(cv::CAP_PROP_FRAME_WIDTH) == 1280);
            REQUIRE(videoCapture.get(cv::CAP_PROP_FRAME_HEIGHT) == 720);
            REQUIRE(videoCapture.get(cv::CAP_PROP_FOURCC) == codec);
            REQUIRE(videoCapture.get(cv::CAP_PROP_FPS) == fps);
            // Read the first frame and check its color values
            cv::Mat frame;
            videoCapture >> frame;
            const auto& color = frame.at<cv::Vec3b>(cv::Point(0, 0));
            // For whatever reasons, OpenCV does not generate the expected blue and red values
            // which were initalliy passed into the dummy bag file. Thus, I had to figure them out
            // manually. These values might change for newer OpenCV versions...
            REQUIRE(static_cast<int>(color[0]) == blueValue);
            REQUIRE(static_cast<int>(color[1]) == greenValue);
            REQUIRE(static_cast<int>(color[2]) == redValue);
        };

        SECTION("Default Parameter Values") {
            thread->start();
            while (!thread->isFinished()) {
            }
            // Codecs are generated out of char sequences, that's why we have these weird numbers
            // Codec number represents mp4v
            performVideoCheck(".mp4", 1983148141, 30, 252, 0, 0);
        }
        SECTION("Modified Parameter Values") {
            parameters.fps = 60;
            parameters.exchangeRedBlueValues = true;

            thread->start();
            while (!thread->isFinished()) {
            }

            performVideoCheck(".mp4", 1983148141, 60, 0, 0, 252);
        }
        SECTION("MKV BW Values") {
            parameters.targetDirectory = "./video.mkv";
            parameters.useBWImages = true;

            thread->start();
            while (!thread->isFinished()) {
            }
            // Codec number represents x264
            performVideoCheck(".mkv", 1734701165, 30, 29, 29, 29);
        }
    }
    SECTION("Video to Bag Thread Test") {
        Parameters::VideoToBagParameters parameters;
        parameters.sourceDirectory = "./video.mp4";
        parameters.targetDirectory = "./video_bag";
        parameters.topicName = "/video_topic";

        rosbag2_cpp::Reader reader;
        rclcpp::Serialization<sensor_msgs::msg::Image> serialization;

        auto* const thread = new VideoToBagThread(parameters);
        QObject::connect(thread, &VideoToBagThread::finished, thread, &QObject::deleteLater);

        const auto performBagCheck = [&reader, &serialization] (unsigned int width, unsigned int height,
                                                                int redValue, int greenValue, int blueValue) {
            reader.open("./video_bag");
            // Check first message values
            auto msg = reader.read_next();
            rclcpp::SerializedMessage serializedMessage(*msg->serialized_data);
            auto rosMsg = std::make_shared<sensor_msgs::msg::Image>();
            serialization.deserialize_message(&serializedMessage, rosMsg.get());
            REQUIRE(rosMsg->width == width);
            REQUIRE(rosMsg->height == height);

            auto cvPointer = cv_bridge::toCvCopy(*rosMsg, rosMsg->encoding);
            const auto& color = cvPointer->image.at<cv::Vec3b>(cv::Point(0, 0));
            REQUIRE(static_cast<int>(color[0]) == redValue);
            REQUIRE(static_cast<int>(color[1]) == greenValue);
            REQUIRE(static_cast<int>(color[2]) == blueValue);

            reader.close();
        };

        SECTION("Default Parameter Values") {
            thread->start();
            while (!thread->isFinished()) {
            }

            const auto& metadata = Utils::ROS::getBagMetadata("./video_bag");
            REQUIRE(metadata.message_count == 200);
            const auto& topics = metadata.topics_with_message_count;
            REQUIRE(topics.size() == 1);
            REQUIRE(topics.at(0).topic_metadata.name == "/video_topic");
            REQUIRE(topics.at(0).topic_metadata.type == "sensor_msgs/msg/Image");
            REQUIRE(topics.at(0).message_count == 200);

            performBagCheck(1280, 720, 0, 0, 252);
        }
        SECTION("Changed Parameter Values") {
            parameters.exchangeRedBlueValues = true;

            thread->start();
            while (!thread->isFinished()) {
            }

            performBagCheck(1280, 720, 252, 0, 0);
        }
        SECTION("MKV Values") {
            parameters.sourceDirectory = "./video.mkv";
            parameters.exchangeRedBlueValues = false;

            // Reset thread to apply new source directory
            if (thread) {
                delete thread;
            }
            auto* const thread = new VideoToBagThread(parameters);
            QObject::connect(thread, &VideoToBagThread::finished, thread, &QObject::deleteLater);

            thread->start();
            while (!thread->isFinished()) {
            }

            performBagCheck(1280, 720, 29, 29, 29);
            std::filesystem::remove("./video.mkv");
            std::filesystem::remove_all("./video_bag");
        }
    }
    SECTION("Bag to Images Thread Test") {
        Parameters::BagToImagesParameters parameters;
        parameters.sourceDirectory = "./dummy_bag";
        parameters.targetDirectory = "./images";
        parameters.topicName = "/dummy_image";

        const auto performImageCheck = [] (const std::string& fileExtension, int valueBlue, int valueGreen, int valueRed) {
            const auto extensionCheckValues = getDirFileCountWithExtensions("./images", fileExtension);
            REQUIRE(extensionCheckValues[0] == 200);
            REQUIRE(extensionCheckValues[1] == 200);

            const auto mat = cv::imread("./images/001" + fileExtension);
            REQUIRE(mat.rows == 720);
            REQUIRE(mat.cols == 1280);
            const auto& color = mat.at<cv::Vec3b>(cv::Point(0, 0));
            REQUIRE(static_cast<int>(color[0]) == valueBlue);
            REQUIRE(static_cast<int>(color[1]) == valueGreen);
            REQUIRE(static_cast<int>(color[2]) == valueRed);
        };

        auto* const thread = new BagToImagesThread(parameters, std::thread::hardware_concurrency());
        QObject::connect(thread, &BagToImagesThread::finished, thread, &QObject::deleteLater);

        SECTION("Default Parameter Values") {
            thread->start();
            while (!thread->isFinished()) {
            }

            performImageCheck(".jpg", 254, 0, 1);
        }
        SECTION("PNG with RB exchanged Values") {
            parameters.format = "png";
            parameters.exchangeRedBlueValues = true;

            thread->start();
            while (!thread->isFinished()) {
            }

            performImageCheck(".png", 1, 0, 255);
        }
        SECTION("BMP with gray exchanged Values") {
            parameters.format = "bmp";
            parameters.useBWImages = true;

            thread->start();
            while (!thread->isFinished()) {
            }

            performImageCheck(".bmp", 29, 29, 29);
        }
    }
    SECTION("Bag to PCDs Thread Test") {
        Parameters::AdvancedParameters parameters;
        parameters.sourceDirectory = "./dummy_bag";
        parameters.targetDirectory = "./pcds";
        parameters.topicName = "/dummy_points";

        auto* const thread = new BagToPCDsThread(parameters, std::thread::hardware_concurrency());
        QObject::connect(thread, &BagToPCDsThread::finished, thread, &QObject::deleteLater);

        thread->start();
        while (!thread->isFinished()) {
        }

        const auto extensionCheckValues = getDirFileCountWithExtensions("./pcds", ".pcd");
        REQUIRE(extensionCheckValues[0] == 200);
        REQUIRE(extensionCheckValues[1] == 200);

        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
        verifyMessages("./dummy_bag", "/dummy_points", serialization, 0);
    }
    SECTION("PCD to Bag Thread Test") {
        Parameters::PCDsToBagParameters parameters;
        parameters.sourceDirectory = "./pcds";
        parameters.targetDirectory = "./bag_pcd";
        parameters.topicName = "/point_clouds_are_awesome";
        parameters.rate = 2;

        auto* const thread = new PCDsToBagThread(parameters, std::thread::hardware_concurrency());
        QObject::connect(thread, &PCDsToBagThread::finished, thread, &QObject::deleteLater);

        thread->start();
        while (!thread->isFinished()) {
        }

        const auto& metaData = Utils::ROS::getBagMetadata("./bag_pcd");
        REQUIRE(metaData.message_count == 200);
        const auto& topics = metaData.topics_with_message_count;
        REQUIRE(topics.size() == 1);
        REQUIRE(topics.at(0).topic_metadata.name == "/point_clouds_are_awesome");
        REQUIRE(topics.at(0).message_count == 200);

        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
        verifyMessages("./bag_pcd", "/point_clouds_are_awesome", serialization, 0);

        std::filesystem::remove_all("./bag_pcd");
        std::filesystem::remove_all("./pcds");
    }
    SECTION("Video as ROS Topic Test") {
        Parameters::PublishParameters parameters;
        parameters.sourceDirectory = "./video.mp4";
        parameters.loop = true;
        parameters.topicName = "/test_publish_video";

        auto run = true;
        cv::Mat mat;

        rclcpp::Rate loopRate(60);
        auto node = std::make_shared<rclcpp::Node>("tests_publisher");
        auto callback = [node, &mat, &run](sensor_msgs::msg::Image::UniquePtr message) {
            if (!run) {
                return;
            }

            auto cvPointer = cv_bridge::toCvCopy(*message, message->encoding);
            mat = cvPointer->image;
            run = false;
        };
        auto subscriber = node->create_subscription<sensor_msgs::msg::Image>("/test_publish_video", 10, callback);

        const auto handleThread = [node, &run, &loopRate, &mat] (QThread* thread, const auto width, const auto height) {
            thread->start();

            while (run) {
                rclcpp::spin_some(node);
                loopRate.sleep();
            }
            // We will just figure out the height and width values, it is impossible
            // to determine certain color values due to the thread running in the background
            REQUIRE(mat.rows == height);
            REQUIRE(mat.cols == width);

            thread->requestInterruption();
            thread->wait();
            delete thread;
        };

        SECTION("Default Parameter Values - Video") {
            auto* thread = new PublishVideoThread(parameters);
            handleThread(thread, 1280, 720);
        }
        SECTION("Scaled Params Values - Video") {
            parameters.scale = true;
            parameters.width = 800;
            parameters.height = 600;

            auto* thread = new PublishVideoThread(parameters);
            handleThread(thread, 800, 600);
        }
        SECTION("Default Parameter Values - Images") {
            parameters.sourceDirectory = "./images";

            auto* thread = new PublishImagesThread(parameters);
            handleThread(thread, 1280, 720);
        }
        SECTION("Scaled Params Values - Image") {
            parameters.sourceDirectory = "./images";
            parameters.scale = true;
            parameters.width = 800;
            parameters.height = 600;

            auto* thread = new PublishImagesThread(parameters);
            handleThread(thread, 800, 600);

            shouldDelete = true;
        }
    }

    // This will be executed before EACH segment, so set true at the very end
    if (shouldDelete) {
        std::filesystem::remove_all("./dummy_bag");
        std::filesystem::remove_all("./edited_bag");
        std::filesystem::remove_all("./images");
        std::filesystem::remove("./video.mp4");
    }
}
