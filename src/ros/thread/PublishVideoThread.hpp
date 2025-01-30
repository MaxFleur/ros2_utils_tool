#pragma once

#include "BasicThread.hpp"
#include "UtilsUI.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

// Thread used to publish a video
// This thread also runs as a separate ROS node to enable the image messages publishing
class PublishVideoThread : public BasicThread {
    Q_OBJECT

public:
    explicit
    PublishVideoThread(const Utils::UI::PublishParameters& parameters,
                       QObject*                            parent = nullptr);

    void
    run() override;

private:
    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_publisher;

    const Utils::UI::PublishParameters& m_parameters;

    static constexpr int PROGRESS = 0;
};
