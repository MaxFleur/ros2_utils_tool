#include "DummyBagThread.hpp"

#include "UtilsROS.hpp"

#include <cv_bridge/cv_bridge.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include <filesystem>
#include <random>

DummyBagThread::DummyBagThread(const Parameters::DummyBagParameters& parameters,
                               unsigned int numberOfThreads, QObject* parent) :
    BasicThread(parameters.sourceDirectory, "", parent),
    m_parameters(parameters), m_numberOfThreads(numberOfThreads)
{
}


void
DummyBagThread::run()
{
    const auto maximumMessageCount = m_parameters.messageCount * m_parameters.topics.size();

    if (std::filesystem::exists(m_sourceDirectory)) {
        std::filesystem::remove_all(m_sourceDirectory);
    }

    auto writer = std::make_shared<rosbag2_cpp::Writer>();
    writer->open(m_sourceDirectory);

    std::deque<Parameters::DummyBagParameters::DummyBagTopic> queue;
    for (const auto& topic : m_parameters.topics) {
        queue.push_front(topic);
    }

    std::mutex mutex;
    std::atomic<int> iterationCount = 1;

    // Move to own lambda for multithreading
    const auto writeDummyTopic = [this, &queue, &mutex, &iterationCount, writer, maximumMessageCount] {
        while (true) {
            mutex.lock();
            if (isInterruptionRequested() || queue.empty()) {
                mutex.unlock();
                break;
            }

            const auto topicType = queue.back().type;
            const auto topicName = queue.back().name.toStdString();
            queue.pop_back();
            mutex.unlock();

            auto timeStamp = rclcpp::Clock(RCL_ROS_TIME).now();
            const auto rate = m_parameters.useCustomRate ? m_parameters.rate : 10;
            const auto duration = rclcpp::Duration::from_seconds(1.0f / static_cast<float>(rate));

            for (auto i = 1; i <= m_parameters.messageCount; i++) {
                if (isInterruptionRequested()) {
                    break;
                }

                timeStamp += duration;
                // Write message depending on type
                if (topicType == "String") {
                    std_msgs::msg::String message;
                    message.data = "Message " + std::to_string(i);

                    writer->write(message, topicName, timeStamp);
                } else if (topicType == "Integer") {
                    std_msgs::msg::Int32 message;
                    message.data = i;

                    writer->write(message, topicName, timeStamp);
                } else if (topicType == "Image") {
                    // Create image which over time is lerped from blue to red
                    const auto blue = 255 - ((i - 1) * (255.0f / static_cast<float>(m_parameters.messageCount)));
                    const auto red = 0 + (i * (255.0f / static_cast<float>(m_parameters.messageCount)));
                    cv::Mat mat(720, 1280, CV_8UC3, cv::Scalar(blue, 0, red));
                    sensor_msgs::msg::Image message;
                    std_msgs::msg::Header header;
                    header.stamp = timeStamp;

                    const auto cvBridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, mat);
                    cvBridge.toImageMsg(message);
                    writer->write(message, topicName, timeStamp);
                } else if (topicType == "Point Cloud" || topicType == "PointCloud") {
                    // Create a randomized point cloud with 10 points
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
                    cloud->width = 10;
                    cloud->height = 1;
                    cloud->is_dense = false;
                    cloud->points.resize(10);

                    std::random_device randomDevice;
                    std::mt19937 generator(randomDevice());
                    std::uniform_real_distribution<> distribution(-1.0, 1.0);
                    for (auto j = 0; j < 10; j++) {
                        (*cloud)[j].x = distribution(generator);
                        (*cloud)[j].y = distribution(generator);
                        (*cloud)[j].z = distribution(generator);
                        (*cloud)[j].r = 255;
                        (*cloud)[j].g = 255;
                        (*cloud)[j].b = 0;
                    }

                    sensor_msgs::msg::PointCloud2 message;
                    pcl::toROSMsg(*cloud, message);
                    writer->write(message, topicName, timeStamp);
                } else {
                    tf2_msgs::msg::TFMessage message;

                    std::random_device randomDevice;
                    std::mt19937 generator(randomDevice());
                    std::uniform_real_distribution<> distribution(-1.0, 1.0);

                    // Generate a tf2 message with three transformations, each containing randomized translation and rotation
                    for (auto i = 0; i < 3; i++) {
                        geometry_msgs::msg::TransformStamped transformStamped;

                        transformStamped.header.stamp = timeStamp;
                        transformStamped.header.frame_id = "world";
                        transformStamped.child_frame_id = "child_tf2";

                        transformStamped.transform.translation.x = distribution(generator);
                        transformStamped.transform.translation.y = distribution(generator);
                        transformStamped.transform.translation.z = distribution(generator);

                        transformStamped.transform.rotation.x = distribution(generator);
                        transformStamped.transform.rotation.y = distribution(generator);
                        transformStamped.transform.rotation.z = distribution(generator);
                        transformStamped.transform.rotation.w = distribution(generator);

                        message.transforms.push_back(transformStamped);
                    }
                    writer->write(message, topicName, timeStamp);
                }

                emit progressChanged("Writing message " + QString::number(iterationCount) + " of " + QString::number(maximumMessageCount) + "...",
                                     (static_cast<float>(iterationCount) / static_cast<float>(maximumMessageCount) * 100));
                iterationCount++;
            }
        }
    };

    // Parallelize the topic writing
    std::vector<std::thread> threadPool;
    for (unsigned int i = 0; i < m_numberOfThreads; ++i) {
        threadPool.emplace_back(writeDummyTopic);
    }
    for (auto& thread : threadPool) {
        thread.join();
    }

    writer->close();
    emit finished();
}
