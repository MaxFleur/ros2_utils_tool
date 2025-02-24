#include "PCDsToBagThread.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include "rosbag2_cpp/writer.hpp"

#include "sensor_msgs/msg/point_cloud.hpp"

#include <filesystem>

PCDsToBagThread::PCDsToBagThread(const Parameters::AdvancedParameters& parameters,
                                 QObject*                              parent) :
    BasicThread(parameters.sourceDirectory, parameters.topicName, parent),
    m_parameters(parameters)
{
}


void
PCDsToBagThread::run()
{
    const auto targetDirectoryStd = m_parameters.targetDirectory.toStdString();
    if (std::filesystem::exists(targetDirectoryStd)) {
        std::filesystem::remove_all(targetDirectoryStd);
    }

    emit informOfGatheringData();

    auto frameCount = 0;
    std::set<std::filesystem::path> sortedPCDsSet;
    for (auto const& entry : std::filesystem::directory_iterator(m_sourceDirectory)) {
        if (entry.path().extension() != ".pcd") {
            continue;
        }

        sortedPCDsSet.insert(entry.path());
        frameCount++;
    }

    auto iterationCount = 1;
    rosbag2_cpp::Writer writer;
    writer.open(targetDirectoryStd);
    std::mutex mutex;

    const auto writeMessageFromQueue = [this, &mutex, &writer, &iterationCount, &sortedPCDsSet, frameCount] {
        while (true) {
            mutex.lock();

            if (isInterruptionRequested() || sortedPCDsSet.empty()) {
                mutex.unlock();
                break;
            }

            sensor_msgs::msg::PointCloud2 message;
            // Nanoseconds directly
            rclcpp::Time time(static_cast<uint64_t>(((float) iterationCount / 2) * 1e9));

            pcl::PCDReader reader;
            pcl::PCLPointCloud2 cloud;
            reader.read(*sortedPCDsSet.begin(), cloud);
            sortedPCDsSet.erase(sortedPCDsSet.begin());

            emit progressChanged("Writing pcd file " + QString::number(iterationCount) + " of " + QString::number(frameCount) + "...",
                                 ((float) iterationCount / (float) frameCount) * 100);
            iterationCount++;

            mutex.unlock();

            pcl_conversions::fromPCL(cloud, message);
            writer.write(message, m_topicName, time);
        }
    };

    std::vector<std::thread> threadPool;
    for (unsigned int i = 0; i < std::thread::hardware_concurrency(); ++i) {
        threadPool.emplace_back(writeMessageFromQueue);
    }
    for (auto& thread : threadPool) {
        thread.join();
    }

    emit finished();
}
