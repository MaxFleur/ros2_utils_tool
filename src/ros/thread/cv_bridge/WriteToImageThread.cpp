#include "WriteToImageThread.hpp"

#include "UtilsROS.hpp"

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/reader.hpp"

#include "sensor_msgs/msg/image.hpp"

#include <filesystem>

#ifdef ROS_JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

WriteToImageThread::WriteToImageThread(const Utils::UI::ImageParameters& imageParameters,
                                       QObject*                          parent) :
    BasicThread(imageParameters.bagDirectory, imageParameters.topicName, parent),
    m_imagesDirectory(imageParameters.imagesDirectory),
    m_format(imageParameters.format), m_quality(imageParameters.quality)
{
}


void
WriteToImageThread::run()
{
    rosbag2_cpp::Reader reader;
    reader.open(m_bagDirectory.toStdString());

    if (!std::filesystem::is_empty(m_imagesDirectory.toStdString())) {
        // Remove all images currently present
        for (const auto& entry : std::filesystem::directory_iterator(m_imagesDirectory.toStdString())) {
            std::filesystem::remove_all(entry.path());
        }
    }

    const auto messageCount = Utils::ROS::getTopicMessageCount(m_bagDirectory.toStdString(), m_topicName.toStdString());
    emit calculatedMaximumInstances(messageCount);

    // Prepare parameters
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
    cv_bridge::CvImagePtr cvPointer;
    auto iterationCount = 0;
    const auto topicNameStdString = m_topicName.toStdString();
    // Adjust the quality value to fit OpenCV param range
    if (m_format == "jpg") {
        m_quality = (m_quality * 10) + 10;
    }

    // Now the main encoding
    while (reader.has_next()) {
        if (isInterruptionRequested()) {
            reader.close();
            return;
        }

        // Read and deserialize the message
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();
        if (msg->topic_name != topicNameStdString) {
            continue;
        }

        rclcpp::SerializedMessage serializedMessage(*msg->serialized_data);
        auto rosMsg = std::make_shared<sensor_msgs::msg::Image>();
        serialization.deserialize_message(&serializedMessage, rosMsg.get());

        // Convert message to cv and encode
        cvPointer = cv_bridge::toCvCopy(*rosMsg, rosMsg->encoding);

        cv::imwrite(m_imagesDirectory.toStdString() + "/" + std::to_string(iterationCount) + "." + m_format.toStdString(), cvPointer->image,
                    { m_format == "jpg" ? cv::IMWRITE_JPEG_QUALITY : cv::IMWRITE_PNG_COMPRESSION, m_quality });

        iterationCount++;
        // Inform of progress update
        emit progressChanged(iterationCount, ((float) iterationCount / (float) messageCount) * 100);
    }

    reader.close();
    emit finished();
}
