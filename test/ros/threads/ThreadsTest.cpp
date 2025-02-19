#include "catch_ros2/catch_ros2.hpp"

#include "DummyBagThread.hpp"
#include "EditBagThread.hpp"
#include "EncodingThread.hpp"
#include "MergeBagsThread.hpp"
#include "PublishImagesThread.hpp"
#include "PublishVideoThread.hpp"
#include "UtilsROS.hpp"
#include "UtilsUI.hpp"
#include "WriteToBagThread.hpp"
#include "WriteToImageThread.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

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


TEST_CASE("Threads Testing", "[threads]") {
    auto shouldDelete = false;

    SECTION("Dummy Bag Thread Test") {
        Utils::UI::DummyBagInputParameters parameters;
        parameters.sourceDirectory = "./dummy_bag";
        parameters.messageCount = 200;
        parameters.topics.push_back({ "Image", "/dummy_image" });
        parameters.topics.push_back({ "Integer", "/dummy_integer" });
        parameters.topics.push_back({ "String", "/dummy_string" });
        parameters.topics.push_back({ "Point Cloud", "/dummy_point_cloud" });

        auto* const thread = new DummyBagThread(parameters);
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
        topicIndex = getTopicIndex(topics, "/dummy_point_cloud");
        REQUIRE(topicIndex < 4);
        REQUIRE(topics.at(topicIndex).topic_metadata.type == "sensor_msgs/msg/PointCloud2");
    }
    // Create edited bag out of dummy bag
    SECTION("Edit Bag Thread Test") {
        Utils::UI::EditBagInputParameters parameters;
        parameters.sourceDirectory = "./dummy_bag";
        parameters.targetDirectory = "./edited_bag";
        parameters.updateTimestamps = true;
        parameters.topics.push_back({ "", "/dummy_image", 0, 100, true });
        parameters.topics.push_back({ "", "/dummy_integer", 0, 200, false });
        parameters.topics.push_back({ "/renamed_string", "/dummy_string", 0, 200, true });

        auto* const thread = new EditBagThread(parameters);
        QObject::connect(thread, &EditBagThread::finished, thread, &QObject::deleteLater);

        thread->start();
        while (!thread->isFinished()) {
        }

        const auto& metadata = Utils::ROS::getBagMetadata("./edited_bag");
        REQUIRE(metadata.message_count == 300);
        const auto& topics = metadata.topics_with_message_count;
        REQUIRE(topics.size() == 2);
        auto topicIndex = getTopicIndex(topics, "/dummy_image");
        REQUIRE(topicIndex < 2);
        REQUIRE(topics.at(topicIndex).topic_metadata.type == "sensor_msgs/msg/Image");
        REQUIRE(topics.at(topicIndex).message_count == 100);
        topicIndex = getTopicIndex(topics, "/renamed_string");
        REQUIRE(topicIndex < 2);
        REQUIRE(topics.at(topicIndex).topic_metadata.type == "std_msgs/msg/String");
        REQUIRE(topics.at(topicIndex).message_count == 200);
    }
    // Merge edited and dummy bag
    SECTION("Merge Bags Thread Test") {
        Utils::UI::MergeBagsInputParameters parameters;
        parameters.sourceDirectory = "./dummy_bag";
        parameters.secondSourceDirectory = "./edited_bag";
        parameters.targetDirectory = "./merged_bag";

        auto* const thread = new MergeBagsThread(parameters);
        QObject::connect(thread, &MergeBagsThread::finished, thread, &QObject::deleteLater);

        SECTION("Unique Topics") {
            parameters.topics.push_back({ "/dummy_integer", "./dummy_bag", true });
            parameters.topics.push_back({ "/renamed_string", "./edited_bag", true });

            thread->start();
            while (!thread->isFinished()) {
            }

            const auto& metadata = Utils::ROS::getBagMetadata("./merged_bag");
            REQUIRE(metadata.message_count == 400);
            const auto& topics = metadata.topics_with_message_count;
            REQUIRE(topics.size() == 2);
            auto topicIndex = getTopicIndex(topics, "/dummy_integer");
            REQUIRE(topicIndex < 2);
            REQUIRE(topics.at(topicIndex).topic_metadata.type == "std_msgs/msg/Int32");
            REQUIRE(topics.at(topicIndex).message_count == 200);
            topicIndex = getTopicIndex(topics, "/renamed_string");
            REQUIRE(topicIndex < 2);
            REQUIRE(topics.at(topicIndex).topic_metadata.type == "std_msgs/msg/String");
            REQUIRE(topics.at(topicIndex).message_count == 200);
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
            REQUIRE(metadata.message_count == 500);
            const auto& topics = metadata.topics_with_message_count;
            REQUIRE(topics.size() == 2);
            auto topicIndex = getTopicIndex(topics, "/renamed_string");
            REQUIRE(topicIndex < 2);
            REQUIRE(topics.at(topicIndex).topic_metadata.type == "std_msgs/msg/String");
            REQUIRE(topics.at(topicIndex).message_count == 200);
            topicIndex = getTopicIndex(topics, "/dummy_image");
            REQUIRE(topicIndex < 2);
            REQUIRE(topics.at(topicIndex).topic_metadata.type == "sensor_msgs/msg/Image");
            REQUIRE(topics.at(topicIndex).message_count == 300);
        }

        std::filesystem::remove_all("./merged_bag");
    }

    SECTION("Encode Video Thread Test") {
        Utils::UI::VideoInputParameters parameters;
        parameters.sourceDirectory = "./dummy_bag";
        parameters.targetDirectory = "./video.mp4";
        parameters.topicName = "/dummy_image";

        auto* const thread = new EncodingThread(parameters);
        QObject::connect(thread, &EncodingThread::finished, thread, &QObject::deleteLater);

        SECTION("Default Parameter Values") {
            thread->start();
            while (!thread->isFinished()) {
            }

            auto videoCapture = cv::VideoCapture("./video.mp4");
            REQUIRE(videoCapture.get(cv::CAP_PROP_FRAME_COUNT) == 200);
            REQUIRE(videoCapture.get(cv::CAP_PROP_FRAME_WIDTH) == 1280);
            REQUIRE(videoCapture.get(cv::CAP_PROP_FRAME_HEIGHT) == 720);
            REQUIRE(videoCapture.get(cv::CAP_PROP_FOURCC) == 1983148141.0);
            REQUIRE(videoCapture.get(cv::CAP_PROP_FPS) == 30);
            // Read the first frame and check its color values
            cv::Mat frame;
            videoCapture >> frame;
            const auto& color = frame.at<cv::Vec3b>(cv::Point(0, 0));
            // Should be mostly blue
            // For whatever reasons, OpenCV does not generate the expected blue and red values
            // which were initalliy passed into the dummy bag file. Thus, I had to figure them out
            // manually. These values might changer for newer OpenCV versions...
            REQUIRE(static_cast<int>(color[0]) == 252);
            REQUIRE(static_cast<int>(color[1]) == 0);
            REQUIRE(static_cast<int>(color[2]) == 0);
        }
        SECTION("Modified Parameter Values") {
            parameters.fps = 60;
            parameters.exchangeRedBlueValues = true;

            thread->start();
            while (!thread->isFinished()) {
            }

            auto videoCapture = cv::VideoCapture("./video.mp4");
            REQUIRE(videoCapture.get(cv::CAP_PROP_FPS) == 60);
            // Read the first frame and check its color values
            cv::Mat frame;
            videoCapture >> frame;
            const auto& color = frame.at<cv::Vec3b>(cv::Point(0, 0));
            // Should be mostly blue
            REQUIRE(static_cast<int>(color[0]) == 0);
            REQUIRE(static_cast<int>(color[1]) == 0);
            REQUIRE(static_cast<int>(color[2]) == 252);
        }
        SECTION("MKV BW Values") {
            parameters.targetDirectory = "./video.mkv";
            parameters.useBWImages = true;

            thread->start();
            while (!thread->isFinished()) {
            }

            auto videoCapture = cv::VideoCapture("./video.mkv");
            REQUIRE(videoCapture.get(cv::CAP_PROP_FOURCC) == 1734701165.0);
            // Read the first frame and check its color values
            cv::Mat frame;
            videoCapture >> frame;
            const auto& color = frame.at<cv::Vec3b>(cv::Point(0, 0));
            // Should be mostly blue
            REQUIRE(static_cast<int>(color[0]) == 29);
            REQUIRE(static_cast<int>(color[1]) == 29);
            REQUIRE(static_cast<int>(color[2]) == 29);
        }
    }
    SECTION("Video to Bag Thread Test") {
        Utils::UI::BagInputParameters parameters;
        parameters.sourceDirectory = "./video.mp4";
        parameters.targetDirectory = "./video_bag";
        parameters.topicName = "/video_topic";

        rosbag2_cpp::Reader reader;
        rclcpp::Serialization<sensor_msgs::msg::Image> serialization;

        auto* const thread = new WriteToBagThread(parameters);
        QObject::connect(thread, &WriteToBagThread::finished, thread, &QObject::deleteLater);

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

            reader.open("./video_bag");
            // Check first message values
            auto msg = reader.read_next();
            rclcpp::SerializedMessage serializedMessage(*msg->serialized_data);
            auto rosMsg = std::make_shared<sensor_msgs::msg::Image>();
            serialization.deserialize_message(&serializedMessage, rosMsg.get());
            REQUIRE(rosMsg->width == 1280);
            REQUIRE(rosMsg->height == 720);

            auto cvPointer = cv_bridge::toCvCopy(*rosMsg, rosMsg->encoding);
            const auto& color = cvPointer->image.at<cv::Vec3b>(cv::Point(0, 0));
            REQUIRE(static_cast<int>(color[0]) == 0);
            REQUIRE(static_cast<int>(color[1]) == 0);
            REQUIRE(static_cast<int>(color[2]) == 252);

            reader.close();
        }
        SECTION("Changed Parameter Values") {
            parameters.exchangeRedBlueValues = true;

            thread->start();
            while (!thread->isFinished()) {
            }

            reader.open("./video_bag");
            // Check first message values
            auto msg = reader.read_next();
            rclcpp::SerializedMessage serializedMessage(*msg->serialized_data);
            auto rosMsg = std::make_shared<sensor_msgs::msg::Image>();
            serialization.deserialize_message(&serializedMessage, rosMsg.get());

            auto cvPointer = cv_bridge::toCvCopy(*rosMsg, rosMsg->encoding);
            const auto& color = cvPointer->image.at<cv::Vec3b>(cv::Point(0, 0));
            REQUIRE(static_cast<int>(color[0]) == 252);
            REQUIRE(static_cast<int>(color[1]) == 0);
            REQUIRE(static_cast<int>(color[2]) == 0);

            reader.close();
        }
        SECTION("MKV Values") {
            parameters.sourceDirectory = "./video.mkv";
            parameters.exchangeRedBlueValues = false;

            // Reset thread to apply new source directory
            if (thread) {
                delete thread;
            }
            auto* const thread = new WriteToBagThread(parameters);
            QObject::connect(thread, &WriteToBagThread::finished, thread, &QObject::deleteLater);

            thread->start();
            while (!thread->isFinished()) {
            }

            reader.open("./video_bag");
            // Check first message values
            auto msg = reader.read_next();
            rclcpp::SerializedMessage serializedMessage(*msg->serialized_data);
            auto rosMsg = std::make_shared<sensor_msgs::msg::Image>();
            serialization.deserialize_message(&serializedMessage, rosMsg.get());

            auto cvPointer = cv_bridge::toCvCopy(*rosMsg, rosMsg->encoding);
            const auto& color = cvPointer->image.at<cv::Vec3b>(cv::Point(0, 0));
            REQUIRE(static_cast<int>(color[0]) == 29);
            REQUIRE(static_cast<int>(color[1]) == 29);
            REQUIRE(static_cast<int>(color[2]) == 29);

            reader.close();

            std::filesystem::remove("./video.mkv");
            std::filesystem::remove_all("./video_bag");
        }
    }
    SECTION("Bag to Images Thread Test") {
        Utils::UI::ImageInputParameters parameters;
        parameters.sourceDirectory = "./dummy_bag";
        parameters.targetDirectory = "./images";
        parameters.topicName = "/dummy_image";

        auto* const thread = new WriteToImageThread(parameters);
        QObject::connect(thread, &WriteToImageThread::finished, thread, &QObject::deleteLater);

        SECTION("Default Parameter Values") {
            thread->start();
            while (!thread->isFinished()) {
            }

            auto numberOfFiles = 0;
            auto numberOfJPGExtension = 0;
            for (const auto& entry : std::filesystem::directory_iterator("./images")) {
                numberOfFiles++;
                if (entry.path().extension() == ".jpg") {
                    numberOfJPGExtension++;
                }
            }
            REQUIRE(numberOfFiles == 200);
            REQUIRE(numberOfJPGExtension == 200);

            const auto mat = cv::imread("./images/001.jpg");
            REQUIRE(mat.rows == 720);
            REQUIRE(mat.cols == 1280);
            // Same thing as described above: No idea why OpenCV generates these values,
            // I had to figure them out manually
            const auto& color = mat.at<cv::Vec3b>(cv::Point(0, 0));
            REQUIRE(static_cast<int>(color[0]) == 254);
            REQUIRE(static_cast<int>(color[1]) == 0);
            REQUIRE(static_cast<int>(color[2]) == 1);
        }
        SECTION("PNG with RB exchanged Values") {
            parameters.format = "png";
            parameters.exchangeRedBlueValues = true;

            thread->start();
            while (!thread->isFinished()) {
            }

            auto numberOfFiles = 0;
            auto numberOfPNGExtension = 0;
            for (const auto& entry : std::filesystem::directory_iterator("./images")) {
                numberOfFiles++;
                if (entry.path().extension() == ".png") {
                    numberOfPNGExtension++;
                }
            }
            REQUIRE(numberOfFiles == 200);
            REQUIRE(numberOfPNGExtension == 200);

            const auto mat = cv::imread("./images/001.png");
            REQUIRE(mat.rows == 720);
            REQUIRE(mat.cols == 1280);
            const auto& color = mat.at<cv::Vec3b>(cv::Point(0, 0));
            REQUIRE(static_cast<int>(color[0]) == 1);
            REQUIRE(static_cast<int>(color[1]) == 0);
            REQUIRE(static_cast<int>(color[2]) == 255);
        }
        SECTION("BMP with gray exchanged Values") {
            parameters.format = "bmp";
            parameters.useBWImages = true;

            thread->start();
            while (!thread->isFinished()) {
            }

            auto numberOfFiles = 0;
            auto numberOfBMPExtension = 0;
            for (const auto& entry : std::filesystem::directory_iterator("./images")) {
                numberOfFiles++;
                if (entry.path().extension() == ".bmp") {
                    numberOfBMPExtension++;
                }
            }
            REQUIRE(numberOfFiles == 200);
            REQUIRE(numberOfBMPExtension == 200);

            const auto mat = cv::imread("./images/001.bmp");
            REQUIRE(mat.rows == 720);
            REQUIRE(mat.cols == 1280);
            const auto& color = mat.at<cv::Vec3b>(cv::Point(0, 0));
            REQUIRE(static_cast<int>(color[0]) == 29);
            REQUIRE(static_cast<int>(color[1]) == 29);
            REQUIRE(static_cast<int>(color[2]) == 29);
        }
    }
    SECTION("Video as ROS Topic Test") {
        Utils::UI::PublishParameters parameters;
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
