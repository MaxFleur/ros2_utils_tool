#include "DummyBagThread.hpp"

#include "UtilsROS.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <filesystem>
#include <random>

#ifdef ROS_JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

DummyBagThread::DummyBagThread(const Utils::UI::DummyBagParameters& parameters,
                               QObject*                             parent) :
    BasicThread(parameters.sourceDirectory, parameters.topicName, parent),
    m_parameters(parameters)
{
}


void
DummyBagThread::run()
{
    const auto maximumMessageCount = m_parameters.messageCount * m_parameters.topics.size();

    if (std::filesystem::exists(m_sourceDirectory)) {
        std::filesystem::remove_all(m_sourceDirectory);
    }

    rosbag2_cpp::Writer writer;
    writer.open(m_sourceDirectory);

    std::atomic<int> iterationCount = 1;

    const auto writeDummyTopic = [this, &writer, &iterationCount, maximumMessageCount] (const auto& type, const auto& name) {
        for (auto i = 1; i <= m_parameters.messageCount; i++) {
            if (isInterruptionRequested()) {
                break;
            }

            const auto timeStamp = rclcpp::Clock().now();

            if (type == "String") {
                Utils::ROS::writeMessageToBag(std_msgs::msg::String(), "Message " + std::to_string(i), writer, name, timeStamp);
            } else if (type == "Integer") {
                Utils::ROS::writeMessageToBag(std_msgs::msg::Int32(), i, writer, name, timeStamp);
            } else if (type == "Image") {
                // Lerp from blue to red
                const auto blue = 255 - ((i - 1) * (255.0f / (float) m_parameters.messageCount));
                const auto red = 0 + (i * (255.0f / (float) m_parameters.messageCount));
                cv::Mat mat(720, 1280, CV_8UC3, cv::Scalar(blue, 0, red));
                sensor_msgs::msg::Image message;
                std_msgs::msg::Header header;
                header.stamp = timeStamp;

                const auto cvBridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, mat);
                cvBridge.toImageMsg(message);
                writer.write(message, name.toStdString(), timeStamp);
            } else if (type == "Point Cloud" || type == "PointCloud") {
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
                writer.write(message, name.toStdString(), timeStamp);
            }

            emit progressChanged("Writing message " + QString::number(iterationCount) + " of " + QString::number(maximumMessageCount) + "...",
                                 ((float) iterationCount / (float) maximumMessageCount) * 100);
            iterationCount++;
        }
    };

    // Parallelize the topic writing
    std::vector<std::thread> threadPool;
    for (const auto& topic : m_parameters.topics) {
        threadPool.emplace_back(writeDummyTopic, topic.type, topic.name);
    }
    for (auto& thread : threadPool) {
        thread.join();
    }

    writer.close();
    emit finished();
}
