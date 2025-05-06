#include "PublishImagesThread.hpp"

#include <opencv2/imgcodecs.hpp>

#ifdef ROS_HUMBLE
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

#include <filesystem>

PublishImagesThread::PublishImagesThread(const Parameters::PublishParameters& parameters,
                                         QObject*                             parent) :
    BasicThread(parameters.sourceDirectory, parameters.topicName, parent),
    m_parameters(parameters)
{
    m_node = std::make_shared<rclcpp::Node>("publish_images");
    m_publisher = m_node->create_publisher<sensor_msgs::msg::Image>(m_topicName, 10);
}


void
PublishImagesThread::run()
{
    rclcpp::Rate rate(m_parameters.fps);
    std::set<std::filesystem::path> sortedImagesSet;
    auto iterator = 0;
    auto frameCount = 0;

    emit informOfGatheringData();
    // It is faster to first store all valid image file paths and then iterate over those
    for (auto const& entry : std::filesystem::directory_iterator(m_sourceDirectory)) {
        if (entry.path().extension() != ".jpg" && entry.path().extension() != ".png" && entry.path().extension() != ".bmp") {
            continue;
        }

        sortedImagesSet.insert(entry.path());
        frameCount++;
    }

    cv::Mat frame;
    sensor_msgs::msg::Image message;

    cv_bridge::CvImage cvBridge;
    cvBridge.encoding = sensor_msgs::image_encodings::BGR8;

    const auto publishImageFiles = [this, &sortedImagesSet, &iterator, &rate, &frame, &message, &cvBridge, frameCount] {
        for (auto const& fileName : sortedImagesSet) {
            if (isInterruptionRequested()) {
                break;
            }
            // Read image from file
            frame = cv::imread(fileName, cv::IMREAD_COLOR);
            if (frame.empty()) {
                continue;
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
            // Spin to publish the next frame
            rclcpp::spin_some(m_node);

            emit progressChanged("Publishing image " + QString::number(iterator + 1) + " of " + QString::number(frameCount) + "...", PROGRESS);
            iterator++;

            rate.sleep();
        }

        iterator = 0;
    };

    do {
        if (isInterruptionRequested()) {
            break;
        }
        publishImageFiles();
    }
    // Loop if set
    while (m_parameters.loop);

    emit finished();
}
