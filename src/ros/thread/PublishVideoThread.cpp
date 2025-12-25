#include "PublishVideoThread.hpp"

#include <opencv2/videoio.hpp>

#ifdef ROS_HUMBLE
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

PublishVideoThread::PublishVideoThread(const Parameters::PublishParameters& parameters, bool useHardwareAcceleration,
                                       QObject*                             parent) :
    BasicThread(parameters.sourceDirectory, parameters.topicName, parent),
    m_parameters(parameters), m_useHardwareAcceleration(useHardwareAcceleration)
{
    m_node = std::make_shared<rclcpp::Node>("publish_video");
    m_publisher = m_node->create_publisher<sensor_msgs::msg::Image>(m_topicName, 10);
}


void
PublishVideoThread::run()
{
    auto videoCapture = cv::VideoCapture(m_sourceDirectory, cv::CAP_ANY, {
        cv::CAP_PROP_HW_ACCELERATION, m_useHardwareAcceleration ? cv::VIDEO_ACCELERATION_ANY : cv::VIDEO_ACCELERATION_NONE
    });
    if (!videoCapture.isOpened()) {
        emit failed();
        return;
    }

    cv::Mat frame;
    sensor_msgs::msg::Image message;

    std_msgs::msg::Header header;
    cv_bridge::CvImage cvBridge;
    cvBridge.header = header;
    cvBridge.encoding = sensor_msgs::image_encodings::BGR8;

    const int rate = ((1000 / static_cast<float>(videoCapture.get(cv::CAP_PROP_FPS))) * 1000);
    auto iterator = 0;
    const auto frameCount = videoCapture.get(cv::CAP_PROP_FRAME_COUNT);

    auto timer = m_node->create_wall_timer(std::chrono::microseconds(rate),
                                           [this, &iterator, &videoCapture, &frame, &message, &cvBridge, frameCount] {
        if (isInterruptionRequested()) {
            return;
        }

        // Loop if set
        if (iterator == frameCount && m_parameters.loop) {
            videoCapture.set(cv::CAP_PROP_POS_FRAMES, 0);
            iterator = 0;
        }

        // Capture image
        videoCapture >> frame;
        if (frame.empty()) {
            return;
        }

        if (m_parameters.exchangeRedBlueValues) {
            cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
        }
        if (m_parameters.scale) {
            cv::resize(frame, frame, cv::Size(m_parameters.width, m_parameters.height), 0, 0);
        }

        // Convert and write image
        cvBridge.image = frame;
        cvBridge.toImageMsg(message);

        m_publisher->publish(message);

        emit progressChanged("Publishing image " + QString::number(iterator + 1) + " of " + QString::number(frameCount) + "...", PROGRESS);
        iterator++;
    });

    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(m_node);

    while (!isInterruptionRequested()) {
        executor->spin_once();
    }

    timer->cancel();
    emit finished();
}
