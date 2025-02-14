#include "PublishImagesThread.hpp"

#include <opencv2/imgcodecs.hpp>

#ifdef ROS_JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <filesystem>

PublishImagesThread::PublishImagesThread(const Utils::UI::PublishParameters& parameters,
                                         QObject*                            parent) :
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

    const auto publishImageFiles = [this, &sortedImagesSet, &iterator, &rate, frameCount] {
        for (auto const& fileName : sortedImagesSet) {
            if (isInterruptionRequested()) {
                break;
            }
            // Read image from file
            auto mat = cv::imread(fileName, cv::IMREAD_COLOR);
            if (mat.empty()) {
                continue;
            }

            if (m_parameters.exchangeRedBlueValues) {
                cv::cvtColor(mat, mat, cv::COLOR_BGR2RGB);
            }
            if (m_parameters.scale) {
                cv::resize(mat, mat, cv::Size(m_parameters.width, m_parameters.height), 0, 0);
            }

            // Create empty sensor message
            sensor_msgs::msg::Image message;
            std_msgs::msg::Header header;
            // Convert and write image
            const auto cvBridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, mat);
            cvBridge.toImageMsg(message);

            m_publisher->publish(message);
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
