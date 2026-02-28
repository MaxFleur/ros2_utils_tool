#include "UtilsROS.hpp"

#include <QRegularExpression>

#include "rosbag2_cpp/reader.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace Utils::ROS
{
void
spinNode(std::shared_ptr<rclcpp::Node> node)
{
    // We spin a node for some time before getting any topics/services from it
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


void
disableROSLogging()
{
    rcutils_logging_set_output_handler([] (const rcutils_log_location_t*, int, const char*,
                                           rcutils_time_point_value_t, const char*, va_list*) {
    });
}


void
sendStaticTransformation(const std::array<double, 3>& translation,
                         const std::array<double, 4>& rotation,
                         std::shared_ptr<NodeWrapper> nodeWrapper)
{
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
    spinNode(node);
}


bool
doesDirectoryContainBagFile(const QString& bagDirectory)
{
    try {
        rosbag2_storage::MetadataIo metadataIO;
        const auto& metadata = metadataIO.read_metadata(bagDirectory.toStdString());
    } catch (...) {
        return false;
    }

    return true;
}


bool
doesDirectoryContainCompressedBagFile(const QString& bagDirectory)
{
    rosbag2_storage::MetadataIo metadata_IO;
    rosbag2_storage::BagMetadata metadata;
    try {
        metadata = metadata_IO.read_metadata(bagDirectory.toStdString());
    } catch (...) {
        return false;
    }

    return metadata.compression_mode == "MESSAGE" || metadata.compression_mode == "FILE";
}


std::vector<std::pair<std::string, std::array<std::string, 3> > >
getTopicInformation()
{
    auto node = std::make_shared<rclcpp::Node>("topics_node");
    spinNode(node);

    std::vector<std::pair<std::string, std::array<std::string, 3> > > topicInformation;
    const auto& currentTopicNamesAndTypes = node->get_topic_names_and_types();
    topicInformation.reserve(currentTopicNamesAndTypes.size());

    for (const auto& currentTopic : currentTopicNamesAndTypes) {
        const auto numberOfPublishers = node->count_publishers(currentTopic.first);
        const auto numberOfSubscribers = node->count_subscribers(currentTopic.first);
        const std::array<std::string, 3> currentTopicInfo { { currentTopic.second.at(0),
            std::to_string(numberOfPublishers), std::to_string(numberOfSubscribers) } };

        topicInformation.emplace_back(std::make_pair(currentTopic.first, currentTopicInfo));
    }

    std::sort(topicInformation.begin(), topicInformation.end());
    return topicInformation;
}


std::map<std::string, std::vector<std::string> >
getServiceNamesAndTypes()
{
    auto node = std::make_shared<rclcpp::Node>("services_node");
    spinNode(node);

    return node->get_service_names_and_types();
}


rosbag2_storage::BagMetadata
getBagMetadata(const QString& bagDirectory)
{
    rosbag2_storage::MetadataIo metadataIO;
    const auto metadata = metadataIO.read_metadata(bagDirectory.toStdString());

    return metadata;
}


std::optional<rosbag2_storage::TopicInformation>
getTopicInBag(const QString& bagDirectory, const QString& topicName)
{
    const auto stdStringTopicName = topicName.toStdString();

    const auto& metadata = getBagMetadata(bagDirectory);
    const auto& topics = metadata.topics_with_message_count;

    auto it = std::ranges::find_if(topics, [&] (const auto& topic) {
        return topic.topic_metadata.name == stdStringTopicName;
    });
    return it != topics.end() ? std::optional(*it) : std::nullopt;
}


bool
doesBagContainTopicName(const QString& bagDirectory, const QString& topicName)
{
    const auto& topic = getTopicInBag(bagDirectory, topicName);
    return topic != std::nullopt;
}


std::optional<int>
getTopicMessageCount(const QString& bagDirectory, const QString& topicName)
{
    const auto& topic = getTopicInBag(bagDirectory, topicName);
    return topic == std::nullopt ? std::nullopt : std::optional(topic->message_count);
}


std::optional<int>
getTopicMessageCount(const std::string& bagDirectory, const std::string& topicName)
{
    return getTopicMessageCount(QString::fromStdString(bagDirectory), QString::fromStdString(topicName));
}


std::optional<QString>
getTopicType(const QString& bagDirectory, const QString& topicName)
{
    const auto& topic = getTopicInBag(bagDirectory, topicName);
    return topic == std::nullopt ? std::nullopt : std::optional(QString::fromStdString(topic->topic_metadata.type));
}


std::optional<QString>
getFirstTopicWithCertainType(const QString& bagDirectory, const QString& typeName)
{
    const auto& metadata = getBagMetadata(bagDirectory);
    const auto& topics = metadata.topics_with_message_count;

    auto it = std::ranges::find_if(topics, [&] (const auto& topic) {
        return topic.topic_metadata.type == typeName.toStdString();
    });
    return it == topics.end() ? std::nullopt : std::optional(QString::fromStdString(it->topic_metadata.name));
}


QVector<QString>
getBagTopics(const QString& bagDirectory, const QString& topicType)
{
    QVector<QString> bagTopics;
    if (const auto doesDirContainBag = doesDirectoryContainBagFile(bagDirectory); !doesDirContainBag) {
        return bagTopics;
    }

    const auto& metadata = getBagMetadata(bagDirectory);
    const auto& topics = metadata.topics_with_message_count;

    for (const auto& topic : topics) {
        if (topic.topic_metadata.type == topicType.toStdString()) {
            bagTopics.push_back(QString::fromStdString(topic.topic_metadata.name));
        }
    }
    return bagTopics;
}


bool
isNameROS2Conform(const QString& topicName)
{
    // Only may contain A-z, a-z, 0-9, _ and /
    QRegularExpression regularExpression("[^A-Za-z0-9/_{}~]");
    if (topicName.contains(regularExpression)) {
        return false;
    }
    // Must not end with /, must not contain __ and //
    if (topicName.endsWith('/') || topicName.contains("//") || topicName.contains("__")) {
        return false;
    }
    // First character must not contain a number
    const auto firstCharacter = QString(topicName.front());
    regularExpression.setPattern("[0-9]");
    if (firstCharacter.contains(regularExpression)) {
        return false;
    }
    // If a ~ is contained, the next character must be a /
    for (auto i = 0; i < topicName.length() - 1; i++) {
        if (topicName.at(i) == '~' && topicName.at(i + 1) != '/') {
            return false;
        }
    }
    // Must have balanced curly braces
    return topicName.count(QLatin1Char('{')) == topicName.count(QLatin1Char('}'));
}
}
