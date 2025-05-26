#include "MergeBagsThread.hpp"

#include "rosbag2_transport/bag_rewrite.hpp"

#include <filesystem>

MergeBagsThread::MergeBagsThread(const Parameters::MergeBagsParameters& parameters,
                                 unsigned int numberOfThreads, QObject* parent) :
    BasicThread(parameters.sourceDirectory, "", parent),
    m_parameters(parameters), m_numberOfThreads(numberOfThreads)
{
}


void
MergeBagsThread::run()
{
    const auto targetDirectoryStd = m_parameters.targetDirectory.toStdString();
    if (std::filesystem::exists(targetDirectoryStd)) {
        std::filesystem::remove_all(targetDirectoryStd);
    }

    // Setup input parameters
    rosbag2_storage::StorageOptions inputStorageFirstBag;
    inputStorageFirstBag.uri = m_sourceDirectory;
    rosbag2_storage::StorageOptions inputStorageSecondBag;
    inputStorageSecondBag.uri = m_parameters.secondSourceDirectory.toStdString();

    // Output parameters
    rosbag2_storage::StorageOptions outputStorage;
    outputStorage.uri = targetDirectoryStd;

    rosbag2_transport::RecordOptions outputRecord;
    for (const auto& topic : m_parameters.topics) {
        if (!topic.isSelected) {
            continue;
        }

        outputRecord.topics.push_back(topic.name.toStdString());
    }

    std::vector<std::pair<rosbag2_storage::StorageOptions, rosbag2_transport::RecordOptions> > outputBags;
    outputBags.push_back({ outputStorage, outputRecord });

    emit processing();
    // Do merge
    rosbag2_transport::bag_rewrite({ inputStorageFirstBag, inputStorageSecondBag }, outputBags);

    if (m_parameters.deleteSource) {
        std::filesystem::remove_all(m_sourceDirectory);
        std::filesystem::remove_all(m_parameters.secondSourceDirectory.toStdString());
    }
    emit finished();
}
