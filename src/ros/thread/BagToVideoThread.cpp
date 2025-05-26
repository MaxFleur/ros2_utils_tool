#include "BagToVideoThread.hpp"

#include "UtilsROS.hpp"
#include "VideoEncoder.hpp"

#include "rosbag2_cpp/reader.hpp"

#include "sensor_msgs/msg/image.hpp"

#ifdef ROS_HUMBLE
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

BagToVideoThread::BagToVideoThread(const Parameters::BagToVideoParameters& parameters, bool useHardwareAcceleration,
                                   QObject*                                parent) :
    BasicThread(parameters.sourceDirectory, parameters.topicName, parent),
    m_parameters(parameters), m_useHardwareAcceleration(useHardwareAcceleration)
{
}


void
BagToVideoThread::run()
{
    rosbag2_cpp::Reader reader;
    reader.open(m_sourceDirectory);

    // Prepare parameters
    const auto messageCount = Utils::ROS::getTopicMessageCount(m_sourceDirectory, m_topicName);
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
    rosbag2_storage::SerializedBagMessageSharedPtr message;
    auto rosMessage = std::make_shared<sensor_msgs::msg::Image>();
    cv_bridge::CvImagePtr cvPointer;

    auto iterationCount = 0;
    const auto topicNameStdString = m_topicName;
    const auto videoEncoder = std::make_shared<VideoEncoder>(m_parameters.format == "mp4" ? cv::VideoWriter::fourcc('m', 'p', '4', 'v') :
                                                             m_parameters.lossless ? cv::VideoWriter::fourcc('F', 'F', 'V', '1')
                                                                                   : cv::VideoWriter::fourcc('X', '2', '6', '4'));

    // Now the main encoding
    while (reader.has_next()) {
        if (isInterruptionRequested()) {
            reader.close();
            return;
        }

        // Read and deserialize the message
        message = reader.read_next();
        if (message->topic_name != topicNameStdString) {
            continue;
        }

        rclcpp::SerializedMessage serializedMessage(*message->serialized_data);
        serialization.deserialize_message(&serializedMessage, rosMessage.get());

        // Setup the video encoder on the first iteration
        if (iterationCount == 0) {
            if (!videoEncoder->setVideoWriter(m_parameters.targetDirectory.toStdString(), m_parameters.fps,
                                              rosMessage->width, rosMessage->height,
                                              m_useHardwareAcceleration, m_parameters.useBWImages)) {
                emit failed();
                return;
            }
        }

        // Convert message to cv and encode
        cvPointer = cv_bridge::toCvCopy(rosMessage, rosMessage->encoding);

        if (m_parameters.useBWImages) {
            // It seems that just setting the VIDEOWRITER_PROP_IS_COLOR in the videowriter leads to a broken video,
            // at least if FFMPEG is used. Converting to a gray mat beforehand provides a fix. More information here:
            // https://github.com/opencv/opencv/issues/26276#issuecomment-2406825667
            cv::Mat greyMat;
            cv::cvtColor(cvPointer->image, greyMat, cv::COLOR_BGR2GRAY);
            videoEncoder->writeImageToVideo(greyMat);
        } else {
            if (m_parameters.exchangeRedBlueValues) {
                cv::cvtColor(cvPointer->image, cvPointer->image, cv::COLOR_BGR2RGB);
            }
            videoEncoder->writeImageToVideo(cvPointer->image);
        }

        iterationCount++;
        emit progressChanged("Writing frame " + QString::number(iterationCount) + " of " + QString::number(*messageCount) + "...",
                             ((float) iterationCount / (float) *messageCount) * 100);
    }

    reader.close();
    emit finished();
}
