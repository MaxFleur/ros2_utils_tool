#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

// Thread used to publish image sequences
// This thread also runs as a separate ROS node to enable the image messages publishing
class PublishImagesThread : public BasicThread {
    Q_OBJECT

public:
    explicit
    PublishImagesThread(const Parameters::PublishParameters& parameters,
                        QObject*                             parent = nullptr);

    void
    run() override;

private:
    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_publisher;

    const Parameters::PublishParameters& m_parameters;

    static constexpr int PROGRESS = 0;
};
