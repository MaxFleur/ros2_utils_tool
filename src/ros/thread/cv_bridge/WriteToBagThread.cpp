#include "WriteToBagThread.hpp"

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"

#include "sensor_msgs/msg/image.hpp"

#include <filesystem>

#ifdef ROS_JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

WriteToBagThread::WriteToBagThread(const Utils::UI::VideoParameters& videoParameters,
                                   QObject*                          parent) :
    BasicThread(videoParameters.bagDirectory, videoParameters.topicName, parent),
    m_videoDirectory(videoParameters.videoDirectory),
    m_useHardwareAcceleration(videoParameters.useHardwareAcceleration)
{
}


void
WriteToBagThread::run()
{
    auto videoCapture = cv::VideoCapture(m_videoDirectory.toStdString(), cv::CAP_ANY, {
        cv::CAP_PROP_HW_ACCELERATION, m_useHardwareAcceleration ? cv::VIDEO_ACCELERATION_ANY : cv::VIDEO_ACCELERATION_NONE
    });
    if (!videoCapture.isOpened()) {
        emit openingCVInstanceFailed();
        return;
    }

    if (std::filesystem::exists(m_bagDirectory.toStdString())) {
        std::filesystem::remove_all(m_bagDirectory.toStdString());
    }

    const auto frameCount = videoCapture.get(cv::CAP_PROP_FRAME_COUNT);
    emit calculatedMaximumInstances(frameCount);

    rosbag2_cpp::Writer writer;
    writer.open(m_bagDirectory.toStdString());
    auto iterationCount = 0;

    auto fps = videoCapture.get(cv::CAP_PROP_FPS);
    if (fps <= 2) {
        fps = 30;
    }

    while (true) {
        if (isInterruptionRequested()) {
            return;
        }

        // Create image
        cv::Mat frame;
        videoCapture >> frame;
        if (frame.empty()) {
            break;
        }

        iterationCount++;

        // Create empty sensor message
        sensor_msgs::msg::Image message;
        std_msgs::msg::Header header;
        const auto seconds = (float) iterationCount / fps;
        const auto time = rclcpp::Time(seconds, seconds * 1000000000);
        header.stamp = time;

        // Convert image and write
        const auto cvBridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame);
        cvBridge.toImageMsg(message);
        writer.write(message, m_topicName.toStdString(), time);

        emit progressChanged(iterationCount, ((float) iterationCount / (float) frameCount) * 100);
    }

    emit finished();
}
