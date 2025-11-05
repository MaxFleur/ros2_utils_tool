#pragma once

#include "rclcpp/rclcpp.hpp"

// Wrapper for a ROS node to keep ROS includes out of the UI code
class NodeWrapper {
public:
    explicit
    NodeWrapper(const std::string& nodeName)
    {
        m_node = std::make_shared<rclcpp::Node>(nodeName);
    }

    std::shared_ptr<rclcpp::Node>
    getNode()
    {
        return m_node;
    }

private:
    std::shared_ptr<rclcpp::Node> m_node;
};
