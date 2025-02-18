#include "DummyBagThread.hpp"

#include "UtilsROS.hpp"

#include "sensor_msgs/msg/image.hpp"

#include <filesystem>

#ifdef ROS_JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

DummyBagThread::DummyBagThread(const Utils::UI::DummyBagInputParameters& parameters,
                               QObject*                                  parent) :
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
