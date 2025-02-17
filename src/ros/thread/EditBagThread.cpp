#include "EditBagThread.hpp"

#include "UtilsROS.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"

#include <filesystem>

EditBagThread::EditBagThread(const Utils::UI::EditBagInputParameters& parameters,
                             QObject*                                 parent) :
    BasicThread(parameters.sourceDirectory, parameters.topicName, parent),
    m_parameters(parameters)
{
}


void
EditBagThread::run()
{
    auto totalInstances = 0;
    for (const auto& topic : m_parameters.topics) {
        if (!topic.isSelected) {
            continue;
        }

        totalInstances += topic.upperBoundary - topic.lowerBoundary;
    }

    emit informOfGatheringData();
    const auto targetDirectoryStd = m_parameters.targetDirectory.toStdString();
    if (std::filesystem::exists(targetDirectoryStd)) {
        std::filesystem::remove_all(targetDirectoryStd);
    }

    rosbag2_cpp::Writer writer;
    writer.open(targetDirectoryStd);
    auto node = std::make_shared<rclcpp::Node>("edit_bag");
    std::atomic<int> instanceCount = 1;
    std::mutex mutex;

    // Move to own lambda for multithreading
    const auto writeTopicToBag = [this, &writer, &instanceCount, &mutex, node, totalInstances] (const auto& topic) {
        const auto originalTopicNameStd = topic.originalTopicName.toStdString();

        mutex.lock();
        const auto& metadata = Utils::ROS::getBagMetadata(m_parameters.sourceDirectory);
        // Create a new topic using either the original or new name
        for (const auto &topicMetaData : metadata.topics_with_message_count) {
            if (topicMetaData.topic_metadata.name == originalTopicNameStd) {
                auto topicToBeModified = topicMetaData.topic_metadata;

                if (!topic.renamedTopicName.isEmpty()) {
                    topicToBeModified.name = topic.renamedTopicName.toStdString();
                }
                writer.create_topic(topicToBeModified);
                break;
            }
        }
        mutex.unlock();

        rosbag2_cpp::Reader reader;
        reader.open(m_sourceDirectory);
        size_t boundaryCounter = 0;

        while (reader.has_next()) {
            if (isInterruptionRequested()) {
                reader.close();
                return;
            }
            // Read the original message
            auto message = reader.read_next();
            if (message->topic_name != originalTopicNameStd) {
                continue;
            }
            // Stay within boundaries
            if (boundaryCounter < topic.lowerBoundary) {
                boundaryCounter++;
                continue;
            }
            if (boundaryCounter == topic.upperBoundary) {
                break;
            }

            if (!topic.renamedTopicName.isEmpty()) {
                message->topic_name = topic.renamedTopicName.toStdString();
            }
            if (m_parameters.updateTimestamps) {
#ifdef ROS_JAZZY
                message->recv_timestamp = node->now().nanoseconds();
#else
                message->time_stamp = node->now().nanoseconds();
#endif
            }
            writer.write(message);

            boundaryCounter++;
            emit progressChanged("Writing message " + QString::number(instanceCount) + " of " + QString::number(totalInstances) + "...",
                                 ((float) instanceCount / (float) totalInstances) * 100);
            instanceCount++;
        }

        reader.close();
    };

    // Parallelize the topic writing
    std::vector<std::thread> threadPool;
    for (const auto& topic : m_parameters.topics) {
        if (!topic.isSelected) {
            continue;
        }

        threadPool.emplace_back(writeTopicToBag, topic);
    }
    for (auto& thread : threadPool) {
        thread.join();
    }

    if (m_parameters.deleteSource) {
        std::filesystem::remove_all(m_sourceDirectory);
    }

    writer.close();
    emit finished();
}
