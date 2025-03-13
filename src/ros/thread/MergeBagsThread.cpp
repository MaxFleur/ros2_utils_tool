#include "MergeBagsThread.hpp"

#include "UtilsROS.hpp"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"

#include <filesystem>

MergeBagsThread::MergeBagsThread(const Parameters::MergeBagsParameters& parameters, int numberOfThreads,
                                 QObject*                               parent) :
    BasicThread(parameters.sourceDirectory, "", parent),
    m_parameters(parameters), m_numberOfThreads(numberOfThreads)
{
}


void
MergeBagsThread::run()
{
    emit informOfGatheringData();

    const auto targetDirectoryStd = m_parameters.targetDirectory.toStdString();
    if (std::filesystem::exists(targetDirectoryStd)) {
        std::filesystem::remove_all(targetDirectoryStd);
    }

    auto totalInstances = 0;
    const auto& metadataFirstBag = Utils::ROS::getBagMetadata(m_parameters.sourceDirectory);
    const auto& metadataSecondBag = Utils::ROS::getBagMetadata(m_parameters.secondSourceDirectory);

    // Add message counts to total instances
    // We need to check that not just the name, but also that bag directory fits (might be that both bags have equal topic names)
    const auto canAddTopicCount = [&totalInstances] (const auto& topic, const auto& metadata, const auto& bagDir) {
        for (const auto &topicMetaData : metadata.topics_with_message_count) {
            if (topicMetaData.topic_metadata.name == topic.name.toStdString() && topic.bagDir.toStdString() == bagDir) {
                totalInstances += topicMetaData.message_count;
                return true;
            }
        }
        return false;
    };

    // Store selected topics in queue
    std::deque<Parameters::MergeBagsParameters::MergeBagTopic> queue;
    for (const auto& topic : m_parameters.topics) {
        if (!topic.isSelected) {
            continue;
        }

        queue.push_front(topic);
        if (canAddTopicCount(topic, metadataFirstBag, m_parameters.sourceDirectory.toStdString())) {
            continue;
        }
        canAddTopicCount(topic, metadataSecondBag, m_parameters.secondSourceDirectory.toStdString());
    }

    rosbag2_cpp::Writer writer;
    writer.open(targetDirectoryStd);
    std::atomic<int> instanceCount = 1;
    std::mutex mutex;

    // Move to own lambda for multithreading
    const auto writeTopicToBag = [this, &queue, &writer, &instanceCount, &mutex, totalInstances] {
        while (true) {
            mutex.lock();

            // Take out and handle items out of the queue until it is empty
            if (isInterruptionRequested() || queue.empty()) {
                mutex.unlock();
                break;
            }

            const auto topic = queue.back();
            queue.pop_back();

            const auto topicNameStd = topic.name.toStdString();
            const auto& metadata = Utils::ROS::getBagMetadata(topic.bagDir);
            // Create a new topic
            for (const auto &topicMetaData : metadata.topics_with_message_count) {
                if (topicMetaData.topic_metadata.name == topicNameStd) {
                    writer.create_topic(topicMetaData.topic_metadata);
                    break;
                }
            }

            rosbag2_cpp::Reader reader;
            reader.open(topic.bagDir.toStdString());
            mutex.unlock();

            while (reader.has_next()) {
                if (isInterruptionRequested()) {
                    reader.close();
                    return;
                }
                // Read the original message
                auto message = reader.read_next();
                if (message->topic_name != topicNameStd) {
                    continue;
                }
                writer.write(message);

                emit progressChanged("Writing message " + QString::number(instanceCount) + " of " + QString::number(totalInstances) + "...",
                                     ((float) instanceCount / (float) totalInstances) * 100);
                instanceCount++;
            }

            reader.close();
        }
    };

    // Parallelize the topic writing
    std::vector<std::thread> threadPool;
    for (int i = 0; i < m_numberOfThreads; ++i) {
        threadPool.emplace_back(writeTopicToBag);
    }
    for (auto& thread : threadPool) {
        thread.join();
    }

    writer.close();
    emit finished();
}
