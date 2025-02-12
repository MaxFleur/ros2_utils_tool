#include "PublishVideoThread.hpp"

#include <opencv2/videoio.hpp>

#ifdef ROS_JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

PublishVideoThread::PublishVideoThread(const Utils::UI::PublishParameters& parameters,
                                       QObject*                            parent) :
    BasicThread(parameters.sourceDirectory, parameters.topicName, parent),
    m_parameters(parameters)
{
    m_node = std::make_shared<rclcpp::Node>("publish_video");
    m_publisher = m_node->create_publisher<sensor_msgs::msg::Image>(m_topicName, 10);
}


void
PublishVideoThread::run()
{
    auto videoCapture = cv::VideoCapture(m_sourceDirectory, cv::CAP_ANY, {
        cv::CAP_PROP_HW_ACCELERATION, m_parameters.useHardwareAcceleration ? cv::VIDEO_ACCELERATION_ANY : cv::VIDEO_ACCELERATION_NONE
    });
    if (!videoCapture.isOpened()) {
        emit openingCVInstanceFailed();
        return;
    }

    rclcpp::Rate rate(videoCapture.get(cv::CAP_PROP_FPS));
    auto iterator = 0;
    const auto frameCount = videoCapture.get(cv::CAP_PROP_FRAME_COUNT);

    while (true) {
        if (isInterruptionRequested()) {
            return;
        }

        // Loop if set
        if (iterator == frameCount && m_parameters.loop) {
            videoCapture.set(cv::CAP_PROP_POS_FRAMES, 0);
            iterator = 0;
        }
        // Capture image
        cv::Mat frame;
        videoCapture >> frame;
        if (frame.empty()) {
            break;
        }

        if (m_parameters.switchRedBlueValues) {
            cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
        }
        if (m_parameters.scale) {
            cv::resize(frame, frame, cv::Size(m_parameters.width, m_parameters.height), 0, 0);
        }

        // Create empty sensor message
        sensor_msgs::msg::Image message;
        std_msgs::msg::Header header;
        // Convert and write image
        const auto cvBridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame);
        cvBridge.toImageMsg(message);

        m_publisher->publish(message);
        rclcpp::spin_some(m_node);

        emit progressChanged("Publishing image " + QString::number(iterator) + " of " + QString::number(frameCount) + "...", PROGRESS);
        iterator++;

        rate.sleep();
    }

    emit finished();
}
