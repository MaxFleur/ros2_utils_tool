#include "ChangeCompressionBagThread.hpp"

#include "rosbag2_transport/bag_rewrite.hpp"

#include <filesystem>

ChangeCompressionBagThread::ChangeCompressionBagThread(const Parameters::CompressBagParameters& parameters,
                                                       int numberOfThreads, bool compress, QObject* parent) :
    BasicThread(parameters.sourceDirectory, parameters.topicName, parent),
    m_parameters(parameters), m_numberOfThreads(numberOfThreads), m_compress(compress)
{
}


void
ChangeCompressionBagThread::run()
{
    const auto targetDirectoryStd = m_parameters.targetDirectory.toStdString();
    if (std::filesystem::exists(targetDirectoryStd)) {
        std::filesystem::remove_all(targetDirectoryStd);
    }

    // Input params
    rosbag2_storage::StorageOptions inputStorage;
    inputStorage.uri = m_sourceDirectory;

    // Output params
    rosbag2_storage::StorageOptions outputStorage;
    outputStorage.uri = targetDirectoryStd;

    rosbag2_transport::RecordOptions outputRecord;
#ifdef ROS_JAZZY
    outputRecord.all_topics = true;
#else
    outputRecord.all = true;
#endif
    if (m_compress) {
        outputRecord.compression_format = "zstd";
        outputRecord.compression_mode = m_parameters.compressPerMessage ? "message" : "file";
        outputRecord.compression_threads = m_numberOfThreads;
        // Need to set this so no messages are dropped
        outputRecord.compression_queue_size = 0;
    }

    std::vector<std::pair<rosbag2_storage::StorageOptions, rosbag2_transport::RecordOptions> > outputBags;
    outputBags.push_back({ outputStorage, outputRecord });

    emit processing();
    // Main rewrite
    rosbag2_transport::bag_rewrite({ inputStorage }, outputBags);

    if (m_parameters.deleteSource) {
        std::filesystem::remove_all(m_sourceDirectory);
    }
    emit finished();
}
