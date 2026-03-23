#include "BagRecorder.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_transport/reader_writer_factory.hpp"
#include "rosbag2_transport/recorder.hpp"

#include <chrono>
#include <filesystem>

BagRecorder::BagRecorder(const Parameters::RecordBagParameters& parameters) : m_parameters(parameters)
{
    const auto targetDirectoryStd = m_parameters.sourceDirectory.toStdString();
    if (std::filesystem::exists(targetDirectoryStd)) {
        std::filesystem::remove_all(targetDirectoryStd);
    }

    // Prepare parameters
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
        m_selectedTopicsCount++;
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

    auto writer = rosbag2_transport::ReaderWriterFactory::make_writer(recordOptions);
    // Initialize recorder
    m_recorder = std::make_shared<rosbag2_transport::Recorder>(std::move(writer), storageOptions, recordOptions);
    m_recorder->record();

    // Need to spin it in an extra thread for writing messages (no idea why, though)
    m_spinThread = std::thread([this] {
        while (!m_abortCalled) {
            rclcpp::spin_some(m_recorder);
            m_rate.sleep();
        }
    });

    m_topicSubscribedFuture = std::async(std::launch::async, std::bind(&BagRecorder::searchForNewSubscription, this));
    m_allTopicsSubscribedFuture = std::async(std::launch::async, std::bind(&BagRecorder::searchForAllSubscriptions, this));
}


void
BagRecorder::searchForNewSubscription()
{
    std::vector<std::string> subscribedTopics {};
    while (true) {
        for (const auto& topic : m_recorder->subscriptions()) {
            if (std::ranges::find(subscribedTopics, topic.first) != subscribedTopics.end()) {
                continue;
            }

            subscribedTopics.push_back(topic.first);
            emit topicSubscribed(QString::fromStdString(topic.first));
        }

        if (m_recorder->subscriptions().size() == m_selectedTopicsCount) {
            break;
        }
        m_topicSubscribedFuture.wait_for(std::chrono::milliseconds(1));
    }
}


void
BagRecorder::searchForAllSubscriptions()
{
    while ((m_selectedTopicsCount != m_recorder->subscriptions().size()) && !m_abortCalled) {
        m_allTopicsSubscribedFuture.wait_for(std::chrono::milliseconds(1));
    }
    emit allTopicsSubscribed();
}


BagRecorder::~BagRecorder()
{
    m_recorder->stop();
    m_abortCalled = true;
    m_spinThread.join();
}
