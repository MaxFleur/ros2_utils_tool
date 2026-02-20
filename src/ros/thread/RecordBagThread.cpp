#include "RecordBagThread.hpp"

#include "rclcpp/rclcpp.hpp"
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

    auto writer = std::make_unique<rosbag2_cpp::Writer>();
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
