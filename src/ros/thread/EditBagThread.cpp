#include "EditBagThread.hpp"

#include "UtilsROS.hpp"
#include "UtilsROSCompression.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_compression/sequential_compression_reader.hpp"
#include "rosbag2_compression/sequential_compression_writer.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"

#include <filesystem>

EditBagThread::EditBagThread(const Parameters::EditBagParameters& parameters, int numberOfThreads,
                             QObject*                             parent) :
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

        totalInstances += topic.upperBoundary - topic.lowerBoundary;
    }

    emit informOfGatheringData();
    const auto targetDirectoryStd = m_parameters.targetDirectory.toStdString();
    if (std::filesystem::exists(targetDirectoryStd)) {
        std::filesystem::remove_all(targetDirectoryStd);
    }

    std::shared_ptr<rosbag2_cpp::Writer> writer;
    if (m_parameters.compressTarget) {
        rosbag2_compression::CompressionOptions compressionOptions;
        compressionOptions.compression_format = "zstd";
        compressionOptions.compression_mode = m_parameters.compressByFile ? rosbag2_compression::CompressionMode::FILE
                                                                          : rosbag2_compression::CompressionMode::MESSAGE;

        auto sequentialWriter = std::make_unique<rosbag2_compression::SequentialCompressionWriter>(compressionOptions);
        writer = std::make_shared<rosbag2_cpp::Writer>(std::move(sequentialWriter));
    } else {
        writer = std::make_shared<rosbag2_cpp::Writer>();
    }
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
    const auto writeTopicToBag = [this, writer, &queue, &instanceCount, &mutex, node, totalInstances] {
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
            const auto& metadata = m_parameters.isSourceCompressed ? Utils::ROS::Compression::getBagMetadata(m_parameters.sourceDirectory)
                                                                   : Utils::ROS::getBagMetadata(m_parameters.sourceDirectory);
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

            std::unique_ptr<rosbag2_cpp::Reader> reader;
            if (m_parameters.isSourceCompressed) {
                auto sequentialCompressionReader = std::make_unique<rosbag2_compression::SequentialCompressionReader>();
                reader = std::make_unique<rosbag2_cpp::Reader>(std::move(sequentialCompressionReader));
            } else {
                reader = std::make_unique<rosbag2_cpp::Reader>();
            }

            reader->open(m_sourceDirectory);
            mutex.unlock();

            size_t boundaryCounter = 0;

            while (reader->has_next()) {
                if (isInterruptionRequested()) {
                    reader->close();
                    return;
                }
                // Read the original message
                auto message = reader->read_next();
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
                writer->write(message);

                emit progressChanged("Writing message " + QString::number(instanceCount) + " of " + QString::number(totalInstances) + "...",
                                     ((float) instanceCount / (float) totalInstances) * 100);
                boundaryCounter++;
                instanceCount++;
            }

            reader->close();
        }
    };

    // Parallelize the topic writing
    std::vector<std::thread> threadPool;
    const unsigned int threadCount = m_parameters.isSourceCompressed ? 1 : std::thread::hardware_concurrency();
    for (unsigned int i = 0; i < threadCount; ++i) {
        threadPool.emplace_back(writeTopicToBag);
    }
    for (auto& thread : threadPool) {
        thread.join();
    }

    if (m_parameters.deleteSource) {
        std::filesystem::remove_all(m_sourceDirectory);
    }

    if (m_parameters.compressTarget) {
        emit compressing();
    }
    writer->close();
    emit finished();
}
