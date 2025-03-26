#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

// Thread used to publish a video as ROS topic
// This thread also runs as a separate ROS node to enable the image messages publishing
class PublishVideoThread : public BasicThread {
    Q_OBJECT

public:
    explicit
    PublishVideoThread(const Parameters::PublishParameters& parameters,
                       bool                                 useHardwareAcceleration,
                       QObject*                             parent = nullptr);

    void
    run() override;

private:
    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_publisher;

    const Parameters::PublishParameters& m_parameters;

    const bool m_useHardwareAcceleration;

    static constexpr int PROGRESS = 0;
};
