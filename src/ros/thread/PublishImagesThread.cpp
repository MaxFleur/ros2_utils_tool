#include "PublishImagesThread.hpp"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgcodecs.hpp>

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
    std::set<std::filesystem::path> sortedImagesSet;
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
    auto setIterator = sortedImagesSet.begin();

    cv::Mat frame;
    sensor_msgs::msg::Image message;

    cv_bridge::CvImage cvBridge;
    cvBridge.encoding = sensor_msgs::image_encodings::BGR8;

    const int rate = ((1000 / static_cast<float>(m_parameters.fps)) * 1000);
    auto iterator = 0;
    auto timer = m_node->create_wall_timer(std::chrono::microseconds(rate),
                                           [this, &iterator, &setIterator, &frame, &cvBridge, &message, sortedImagesSet, frameCount] {
        if (isInterruptionRequested()) {
            return;
        }

        // Loop if set
        if (iterator == frameCount) {
            if (m_parameters.loop) {
                setIterator = sortedImagesSet.begin();
                iterator = 0;
            } else {
                return;
            }
        }

        // Read image from file
        frame = cv::imread(*setIterator, cv::IMREAD_COLOR);
        if (frame.empty()) {
            iterator++;
            setIterator++;
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
        setIterator++;
    });

    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(m_node);

    while (!isInterruptionRequested()) {
        executor->spin_once();
    }

    timer->cancel();
    emit finished();
}
