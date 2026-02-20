#include "RecordBagThread.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_transport/reader_writer_factory.hpp"
#include "rosbag2_transport/recorder.hpp"

#include <filesystem>

RecordBagThread::RecordBagThread(const Parameters::RecordBagParameters& parameters, QObject* parent) :
    BasicThread(parameters.sourceDirectory, "", parent), m_parameters(parameters)
{
}


void
RecordBagThread::run()
{
    const auto targetDirectoryStd = m_parameters.sourceDirectory.toStdString();
    if (std::filesystem::exists(targetDirectoryStd)) {
        std::filesystem::remove_all(targetDirectoryStd);
    }

    rosbag2_storage::StorageOptions storageOptions;
    storageOptions.uri = targetDirectoryStd;
    storageOptions.max_bagfile_size = m_parameters.useCustomSize ? m_parameters.maxSizeInMB * 1048576 : 0;
    storageOptions.max_bagfile_duration = m_parameters.useCustomDuration ? m_parameters.maxDurationInSeconds : 0;

    rosbag2_transport::RecordOptions recordOptions;
    for (const auto& topic : m_parameters.topics) {
        if (!topic.isSelected) {
            continue;
        }
        recordOptions.topics.push_back(topic.name.toStdString());
    }
    recordOptions.rmw_serialization_format = "cdr";
    recordOptions.include_hidden_topics = m_parameters.includeHiddenTopics;
    recordOptions.include_unpublished_topics = m_parameters.includeUnpublishedTopics;
    if (m_parameters.useCompression) {
        recordOptions.compression_format = "zstd";
        recordOptions.compression_mode = m_parameters.isCompressionFile ? "file" : "message";
        // Need to set this so no messages are dropped
        recordOptions.compression_queue_size = 0;
    }

    // Disable terminal logging
    auto result = rcutils_logging_set_logger_level("rosbag2_recorder", RCUTILS_LOG_SEVERITY_FATAL);
    Q_UNUSED(result);
    auto writer = rosbag2_transport::ReaderWriterFactory::make_writer(recordOptions);
    auto recorder = std::make_shared<rosbag2_transport::Recorder>(std::move(writer), storageOptions, recordOptions);

    // Initialize recorder
    recorder->record();
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(recorder);

    // Need to spin it in an extra thread for writing messages (no idea why, though)
    auto spinThread = std::thread([executor] {
        executor->spin();
    });

    rclcpp::Rate rate(50);
    while (!isInterruptionRequested()) {
        rate.sleep();
    }

    recorder->stop();
    executor->cancel();
    spinThread.join();

    emit finished();
}
