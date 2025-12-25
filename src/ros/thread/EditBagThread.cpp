#include "EditBagThread.hpp"

#include "UtilsROS.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"

#include <filesystem>

EditBagThread::EditBagThread(const Parameters::EditBagParameters& parameters,
                             unsigned int numberOfThreads, QObject* parent) :
    BasicThread(parameters.sourceDirectory, parameters.topicName, parent),
    m_parameters(parameters), m_numberOfThreads(numberOfThreads)
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

        totalInstances += topic.upperBoundary - topic.lowerBoundary + 1;
    }

    emit informOfGatheringData();
    const auto targetDirectoryStd = m_parameters.targetDirectory.toStdString();
    if (std::filesystem::exists(targetDirectoryStd)) {
        std::filesystem::remove_all(targetDirectoryStd);
    }

    auto writer = std::make_shared<rosbag2_cpp::Writer>();
    writer->open(targetDirectoryStd);

    // Store selected topics in queue
    std::deque<Parameters::EditBagParameters::EditBagTopic> queue;
    for (const auto& topic : m_parameters.topics) {
        if (!topic.isSelected) {
            continue;
        }

        queue.push_front(topic);
    }

    auto node = std::make_shared<rclcpp::Node>("edit_bag");
    std::atomic<int> instanceCount = 1;
    std::mutex mutex;

    // Move to own lambda for multithreading
    const auto writeTopicToBag = [this, &queue, &instanceCount, &mutex, writer, node, totalInstances] {
        while (true) {
            mutex.lock();

            // Take out and handle items out of the queue until it is empty
            if (isInterruptionRequested() || queue.empty()) {
                mutex.unlock();
                break;
            }
            const auto topic = queue.back();
            queue.pop_back();

            const auto originalTopicNameStd = topic.originalTopicName.toStdString();
            const auto& metadata = Utils::ROS::getBagMetadata(m_parameters.sourceDirectory);
            // Create a new topic using either the original or new name
            for (const auto &topicMetaData : metadata.topics_with_message_count) {
                if (topicMetaData.topic_metadata.name == originalTopicNameStd) {
                    auto topicToBeModified = topicMetaData.topic_metadata;

                    if (!topic.renamedTopicName.isEmpty()) {
                        topicToBeModified.name = topic.renamedTopicName.toStdString();
                    }
                    writer->create_topic(topicToBeModified);
                    break;
                }
            }

            rosbag2_cpp::Reader reader;
            reader.open(m_sourceDirectory);
            mutex.unlock();

            rosbag2_storage::SerializedBagMessageSharedPtr message;
            size_t boundaryCounter = 0;

            while (reader.has_next()) {
                if (isInterruptionRequested()) {
                    reader.close();
                    return;
                }
                // Read the original message
                message = reader.read_next();
                if (message->topic_name != originalTopicNameStd) {
                    continue;
                }
                // Stay within boundaries
                if (boundaryCounter < topic.lowerBoundary) {
                    boundaryCounter++;
                    continue;
                }
                if (boundaryCounter > topic.upperBoundary) {
                    break;
                }

                if (!topic.renamedTopicName.isEmpty()) {
                    message->topic_name = topic.renamedTopicName.toStdString();
                }
                if (m_parameters.updateTimestamps) {
#ifdef ROS_HUMBLE
                    message->time_stamp = node->now().nanoseconds();
#else
                    message->recv_timestamp = node->now().nanoseconds();
#endif
                }
                writer->write(message);

                emit progressChanged("Writing message " + QString::number(instanceCount) + " of " + QString::number(totalInstances) + "...",
                                     (static_cast<float>(instanceCount) / static_cast<float>(totalInstances)) * 100);
                boundaryCounter++;
                instanceCount++;
            }

            reader.close();
        }
    };

    // Parallelize the topic writing
    std::vector<std::thread> threadPool;
    for (unsigned int i = 0; i < m_numberOfThreads; ++i) {
        threadPool.emplace_back(writeTopicToBag);
    }
    for (auto& thread : threadPool) {
        thread.join();
    }

    if (m_parameters.deleteSource) {
        std::filesystem::remove_all(m_sourceDirectory);
    }

    writer->close();
    emit finished();
}
