#pragma once

#include "BasicThread.hpp"
#include "Parameters.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

// Thread used to publish transformations. Runs using a ROS node to enable publishing
class SendTF2Thread : public BasicThread {
    Q_OBJECT

public:
    explicit
    SendTF2Thread(const Parameters::SendTF2Parameters& parameters,
                  QObject*                             parent = nullptr);

    void
    run() override;

private:
    std::shared_ptr<rclcpp::Node> m_node;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_broadcaster;

    const Parameters::SendTF2Parameters& m_parameters;

    static constexpr int PROGRESS = 0;
};
