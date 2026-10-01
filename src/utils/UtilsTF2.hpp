#pragma once

#include "NodeWrapper.hpp"

#include "tf2_ros/static_transform_broadcaster.h"

// Util functions for tf2 transformations
namespace Utils::TF2
{
// Keep these header-only to prevent globalized tf2_ros imports for all tools using utils

// Sends a static transformation using tf broadcaster
// @NOTE: For whatever reason, just creating and spinning a local node
//        does not work here, we have to spin a global node
inline void
sendStaticTransformation(const std::array<double, 3>& translation,
                         const std::array<double, 4>& rotation,
                         std::shared_ptr<NodeWrapper> nodeWrapper)
{
    // We need to create and spin a node for some time to be able to send transformations
    auto node = nodeWrapper->getNode();
    auto broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = node->get_clock()->now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "tf_test";

    transformStamped.transform.translation.x = translation[0];
    transformStamped.transform.translation.y = translation[1];
    transformStamped.transform.translation.z = translation[2];

    transformStamped.transform.rotation.x = rotation[0];
    transformStamped.transform.rotation.y = rotation[1];
    transformStamped.transform.rotation.z = rotation[2];
    transformStamped.transform.rotation.w = rotation[3];

    broadcaster->sendTransform(transformStamped);

    // This implementation is based is based on ros2cli:
    // https://github.com/ros2/ros2cli/blob/rolling/ros2cli/ros2cli/node/direct.py#L25
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);

    rclcpp::Rate rate(50);
    auto isFinished = false;

    auto timer = rclcpp::create_timer(node, node->get_clock(), rclcpp::Duration::from_seconds(0.1), [&isFinished] {
        isFinished = true;
    });
    while (!isFinished) {
        executor->spin_once();
        rate.sleep();
    }
}
}
