#include "EditBagThread.hpp"

#include "UtilsROS.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"

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
    // Calculate number of messages written
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

    // Prepare parameters
    rosbag2_storage::StorageOptions storageOptions;
    storageOptions.uri = targetDirectoryStd;

    std::shared_ptr<rosbag2_cpp::Writer> writer;

    if (m_parameters.compressTarget) {
        rosbag2_transport::RecordOptions recordOptions;
        recordOptions.rmw_serialization_format = "cdr";
        recordOptions.compression_format = "zstd";
        recordOptions.compression_mode = m_parameters.compressPerMessage ? "message" : "file";
        recordOptions.compression_threads = m_numberOfThreads;
        // Need to set this to prevent message dropping
        recordOptions.compression_queue_size = 0;

        writer = rosbag2_transport::ReaderWriterFactory::make_writer(recordOptions);
    } else {
        writer = std::make_shared<rosbag2_cpp::Writer>();
    }

    // The factory returns a writer with a compression-enabled implementation
    writer->open(storageOptions);

    // Store selected topics in queue so they can be accessed in parallel
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

    // Our input might be compressed. In this case, ROS2 generates a sequential compression reader,
    // which generates a decompressed bag file. If we had multiple of these, every reader would generate
    // the decompressed file again while the writer is still running, possibly interrupting it for this topic.
    // A safer way here is to disable the parallel approach for compressed files.
    if (Utils::ROS::doesDirectoryContainCompressedBagFile(m_parameters.sourceDirectory)) {
        m_numberOfThreads = 1;
    }

    // Move to own lambda for multithreading
    const auto writeTopicToBag = [this, &queue, &instanceCount, &mutex, writer, node, totalInstances] {
        while (true) {
            // Reader can't work in parallel, so we have to lock this part
            mutex.lock();

            // Take out and handle items out of the queue until it is empty or the thread was stopped
            if (isInterruptionRequested() || queue.empty()) {
                mutex.unlock();
                break;
            }
            const auto topic = queue.back();
            queue.pop_back();

            const auto nameStd = topic.name.toStdString();
            const auto& metadata = Utils::ROS::getBagMetadata(m_parameters.sourceDirectory);

            // Create a new topic using either the original or new name
            for (const auto &topicMetaData : metadata.topics_with_message_count) {
                if (topicMetaData.topic_metadata.name != nameStd) {
                    continue;
                }

                auto topicToBeModified = topicMetaData.topic_metadata;
                if (!topic.renamedName.isEmpty()) {
                    topicToBeModified.name = topic.renamedName.toStdString();
                }

                writer->create_topic(topicToBeModified);
                break;
            }

            rosbag2_storage::StorageOptions inputStorageOptions;
            inputStorageOptions.uri = m_sourceDirectory;

            auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(inputStorageOptions);
            reader->open(inputStorageOptions);
            mutex.unlock();

            rosbag2_storage::SerializedBagMessageSharedPtr message;
            size_t boundaryCounter = 0;

            while (reader->has_next()) {
                if (isInterruptionRequested()) {
                    reader->close();
                    return;
                }
                // Read the original message
                message = reader->read_next();
                if (message->topic_name != nameStd) {
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

                // Set data and write
                if (!topic.renamedName.isEmpty()) {
                    message->topic_name = topic.renamedName.toStdString();
                }
                if (m_parameters.updateTimestamps) {
                    message->recv_timestamp = node->now().nanoseconds();
                }
                writer->write(message);

                emit progressChanged("Writing Message " + QString::number(instanceCount) + " of " + QString::number(totalInstances) + "...",
                                     (static_cast<float>(instanceCount) / static_cast<float>(totalInstances)) * 100);
                boundaryCounter++;
                instanceCount++;
            }

            reader->close();
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
