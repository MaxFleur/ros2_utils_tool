#pragma once

#include <QString>
#include <QVector>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/bag_metadata.hpp"

#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

#include <string>

template<typename T, typename U>
concept WriteMessageParameter = (std::same_as<T, std_msgs::msg::String> && std::same_as<U, std::string>) ||
                                (std::same_as<T, std_msgs::msg::Int32> && std::same_as<U, int>);

// ROS related util functions
namespace Utils::ROS
{
template<typename T, typename U>
requires WriteMessageParameter<T, U>
void
writeMessageToBag(T                    message,
                  const U              messageData,
                  rosbag2_cpp::Writer& writer,
                  const QString&       topicName,
                  const rclcpp::Time&  timeStamp)
{
    message.data = messageData;
    writer.write(message, topicName.toStdString(), timeStamp);
}


// If a directory contains a valid ROS bag
[[nodiscard]] bool
doesDirectoryContainBagFile(const QString& bagDirectory);

// If a ROS bag contains a certain topic
[[nodiscard]] bool
doesBagContainTopicName(const QString& bagDirectory,
                        const QString& topicName);

// Message count for a ROS bag topic
[[nodiscard]] int
getTopicMessageCount(const QString& bagDirectory,
                     const QString& topicName);

[[nodiscard]] int
getTopicMessageCount(const std::string& bagDirectory,
                     const std::string& topicName);

// Returns the metadata stored for a ROS bag
[[nodiscard]] rosbag2_storage::BagMetadata
getBagMetadata(const QString& bagDirectory);

// Get a ROS bag topic's type
[[nodiscard]] QString
getTopicType(const QString& bagDirectory,
             const QString& topicName);

// Returns the first topic in a bag file with a certain type
[[nodiscard]] std::optional<QString>
getFirstTopicWithCertainType(const QString& bagDirectory,
                             const QString& typeName);

// Returns all video bag topics stored in a ROS bag file
[[nodiscard]] QVector<QString>
getBagTopics(const QString& bagDirectory,
             const QString& topicType);

// Returns if a topic name follows the ROS2 naming convention
[[nodiscard]] bool
isNameROS2Conform(const QString& topicName);
}
