#pragma once

#include "rclcpp/rclcpp.hpp"

// Wrapper for a ROS node to keep ROS includes out of UI code.
// Single registry, so requests for the same name return the same node.
class NodeWrapper {
public:
    explicit
    NodeWrapper(const std::string& nodeName)
    {
        m_node = getOrCreate(nodeName);
    }

    std::shared_ptr<rclcpp::Node>
    getNode()
    {
        return m_node;
    }

    static std::shared_ptr<rclcpp::Node>
    getOrCreate(const std::string& nodeName)
    {
        static std::mutex mutex;
        static std::unordered_map<std::string, std::weak_ptr<rclcpp::Node> > nodes;

        // Node already exists
        std::lock_guard<std::mutex> lock(mutex);
        if (const auto existing = nodes[nodeName].lock()) {
            return existing;
        }

        // Node does not exist yet
        auto node = std::make_shared<rclcpp::Node>(nodeName);
        nodes[nodeName] = node;
        return node;
    }

private:
    std::shared_ptr<rclcpp::Node> m_node;
};
