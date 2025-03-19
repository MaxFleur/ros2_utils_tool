#include "CompressBagThread.hpp"

#include "UtilsROS.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_compression/sequential_compression_writer.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"

#include <filesystem>

CompressBagThread::CompressBagThread(const Parameters::CompressBagParameters& parameters, int numberOfThreads,
                                     QObject*                                 parent) :
    BasicThread(parameters.sourceDirectory, parameters.topicName, parent),
    m_parameters(parameters), m_numberOfThreads(numberOfThreads)
{
}


void
CompressBagThread::run()
{
    const auto targetDirectoryStd = m_parameters.targetDirectory.toStdString();
    if (std::filesystem::exists(targetDirectoryStd)) {
        std::filesystem::remove_all(targetDirectoryStd);
    }

    auto totalInstances = 0;
    std::deque<std::string> queue;

    const auto& metadata = Utils::ROS::getBagMetadata(m_parameters.sourceDirectory);
    for (const auto &topicMetaData : metadata.topics_with_message_count) {
        totalInstances += topicMetaData.message_count;
        queue.push_front(topicMetaData.topic_metadata.name);
    }

    rosbag2_compression::CompressionOptions compressionOptions;
    compressionOptions.compression_format = "zstd";
    compressionOptions.compression_mode = m_parameters.compressPerMessage ? rosbag2_compression::CompressionMode::MESSAGE :
                                          rosbag2_compression::CompressionMode::FILE;
    auto writer = std::make_shared<rosbag2_compression::SequentialCompressionWriter>(compressionOptions);

    rosbag2_storage::StorageOptions storageOptions;
    storageOptions.uri = m_parameters.targetDirectory.toStdString();
    writer->open(storageOptions, rosbag2_cpp::ConverterOptions());

    std::atomic<int> instanceCount = 1;
    std::mutex mutex;

    // Move to own lambda for multithreading
    const auto writeTopicToBag = [this, &queue, &mutex, &instanceCount, &metadata, writer, totalInstances] {
        while (true) {
            mutex.lock();

            // Take out and handle items out of the queue until it is empty
            if (isInterruptionRequested() || queue.empty()) {
                mutex.unlock();
                break;
            }

            const auto topic = queue.back();
            queue.pop_back();

            // Create a new topic
            for (const auto &topicMetaData : metadata.topics_with_message_count) {
                if (topicMetaData.topic_metadata.name == topic) {
                    writer->create_topic(topicMetaData.topic_metadata);
                    break;
                }
            }

            rosbag2_cpp::Reader reader;
            reader.open(m_sourceDirectory);
            mutex.unlock();

            while (reader.has_next()) {
                if (isInterruptionRequested()) {
                    reader.close();
                    return;
                }
                // Read the original message
                auto message = reader.read_next();
                if (message->topic_name != topic) {
                    continue;
                }

                writer->write(message);

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

    if (isInterruptionRequested()) {
        return;
    }

    emit compressing();
    writer->close();

    if (m_parameters.deleteSource) {
        std::filesystem::remove_all(m_sourceDirectory);
    }
    emit finished();
}
