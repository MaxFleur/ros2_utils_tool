#include "UtilsROS.hpp"

#include <QRegularExpression>

#include "rosbag2_cpp/reader.hpp"

namespace Utils::ROS
{
bool
doesDirectoryContainBagFile(const QString& bagDirectory)
{
    rosbag2_cpp::Reader reader;
    try {
        reader.open(bagDirectory.toStdString());
    } catch (...) {
        return false;
    }

    reader.close();
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


bool
doesBagContainTopicName(const QString& bagDirectory, const QString& topicName)
{
    rosbag2_cpp::Reader reader;
    reader.open(bagDirectory.toStdString());

    const auto& topicsAndTypes = reader.get_all_topics_and_types();
    const auto stdStringTopicName = topicName.toStdString();

    auto iter = std::find_if(topicsAndTypes.begin(), topicsAndTypes.end(), [&] (const auto& topic) {
        return topic.name == stdStringTopicName;
    });

    reader.close();
    return iter != topicsAndTypes.end();
}


int
getTopicMessageCount(const QString& bagDirectory, const QString& topicName)
{
    rosbag2_cpp::Reader reader;
    reader.open(bagDirectory.toStdString());

    const auto& topics = reader.get_metadata().topics_with_message_count;
    const auto stdStringTopicName = topicName.toStdString();

    auto iter = std::find_if(topics.begin(), topics.end(), [&] (const auto& topic) {
        return topic.topic_metadata.name == stdStringTopicName;
    });

    reader.close();
    return iter != topics.end() ? iter->message_count : 0;
}


int
getTopicMessageCount(const std::string& bagDirectory, const std::string& topicName)
{
    return getTopicMessageCount(QString::fromStdString(bagDirectory), QString::fromStdString(topicName));
}


rosbag2_storage::BagMetadata
getBagMetadata(const QString& bagDirectory)
{
    rosbag2_cpp::Reader reader;

    reader.open(bagDirectory.toStdString());
    const auto metaData = reader.get_metadata();
    reader.close();

    return metaData;
}


QString
getTopicType(const QString& bagDirectory, const QString& topicName)
{
    rosbag2_cpp::Reader reader;
    reader.open(bagDirectory.toStdString());

    const auto& topicsAndTypes = reader.get_all_topics_and_types();
    const auto stdStringTopicName = topicName.toStdString();

    auto iter = std::find_if(topicsAndTypes.begin(), topicsAndTypes.end(), [&] (const auto& topic) {
        return topic.name == stdStringTopicName;
    });

    reader.close();
    return iter != topicsAndTypes.end() ? QString::fromStdString(iter->type) : "";
}


std::optional<QString>
getFirstTopicWithCertainType(const QString& bagDirectory, const QString& typeName)
{
    const auto& bagMetaData = Utils::ROS::getBagMetadata(bagDirectory);
    const auto& topics = bagMetaData.topics_with_message_count;

    auto it = std::find_if(topics.begin(), topics.end(), [&] (const auto& topic) {
        return topic.topic_metadata.type == typeName.toStdString();
    });
    if (it == topics.end()) {
        return std::nullopt;
    }

    return QString::fromStdString(it->topic_metadata.name);
}


QVector<QString>
getBagTopics(const QString& bagDirectory, const QString& topicType)
{
    QVector<QString> videoTopics;
    if (const auto doesDirContainBag = doesDirectoryContainBagFile(bagDirectory); !doesDirContainBag) {
        return videoTopics;
    }

    rosbag2_cpp::Reader reader;
    reader.open(bagDirectory.toStdString());

    for (const auto topicsAndTypes = reader.get_all_topics_and_types(); const auto& topicAndType : topicsAndTypes) {
        if (topicAndType.type == topicType.toStdString()) {
            videoTopics.push_back(QString::fromStdString(topicAndType.name));
        }
    }
    reader.close();
    return videoTopics;
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
