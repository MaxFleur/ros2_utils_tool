#include "VideoToBagThread.hpp"

#include <opencv2/videoio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <filesystem>

#ifdef ROS_HUMBLE
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

VideoToBagThread::VideoToBagThread(const Parameters::VideoToBagParameters& parameters, bool useHardwareAcceleration,
                                   QObject*                                parent) :
    BasicThread(parameters.sourceDirectory, parameters.topicName, parent),
    m_parameters(parameters), m_useHardwareAcceleration(useHardwareAcceleration)
{
}


void
VideoToBagThread::run()
{
    auto videoCapture = cv::VideoCapture(m_sourceDirectory, cv::CAP_ANY, {
        cv::CAP_PROP_HW_ACCELERATION, m_useHardwareAcceleration ? cv::VIDEO_ACCELERATION_ANY : cv::VIDEO_ACCELERATION_NONE
    });
    if (!videoCapture.isOpened()) {
        emit failed();
        return;
    }

    const auto targetDirectoryStd = m_parameters.targetDirectory.toStdString();
    if (std::filesystem::exists(targetDirectoryStd)) {
        std::filesystem::remove_all(targetDirectoryStd);
    }

    const auto frameCount = videoCapture.get(cv::CAP_PROP_FRAME_COUNT);
    const auto finalFPS = m_parameters.useCustomFPS ? m_parameters.fps : videoCapture.get(cv::CAP_PROP_FPS);
    auto timeStamp = rclcpp::Clock(RCL_ROS_TIME).now();
    const auto duration = rclcpp::Duration::from_seconds(1.0f / (float) finalFPS);

    auto writer = std::make_unique<rosbag2_cpp::Writer>();
    writer->open(targetDirectoryStd);
    auto iterationCount = 0;

    cv::Mat frame;
    std_msgs::msg::Header header;
    sensor_msgs::msg::Image message;

    cv_bridge::CvImage cvBridge;
    cvBridge.header = header;
    cvBridge.encoding = sensor_msgs::image_encodings::BGR8;

    while (true) {
        if (isInterruptionRequested()) {
            writer->close();
            return;
        }

        // Capture image
        videoCapture >> frame;
        if (frame.empty()) {
            break;
        }

        iterationCount++;
        if (m_parameters.exchangeRedBlueValues) {
            cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
        }

        // Create empty sensor message
        timeStamp += duration;
        header.stamp = timeStamp;

        // Convert and write image
        cvBridge.image = frame;
        cvBridge.toImageMsg(message);
        writer->write(message, m_topicName, timeStamp);

        emit progressChanged("Writing message " + QString::number(iterationCount) + " of " + QString::number(frameCount) + "...",
                             ((float) iterationCount / (float) frameCount) * 100);
    }

    writer->close();
    emit finished();
}
