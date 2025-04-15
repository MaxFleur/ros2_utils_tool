#include "PCDsToBagThread.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include "rosbag2_cpp/writer.hpp"

#include "sensor_msgs/msg/point_cloud.hpp"

#include <filesystem>

PCDsToBagThread::PCDsToBagThread(const Parameters::PCDsToBagParameters& parameters, QObject* parent) :
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
    // It is faster to first store all valid pcd file paths and then iterate over those
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

    auto timeStamp = rclcpp::Clock(RCL_ROS_TIME).now();
    const auto duration = rclcpp::Duration::from_seconds(1.0f / (float) m_parameters.rate);

    while (true) {
        if (isInterruptionRequested() || sortedPCDsSet.empty()) {
            break;
        }
        // Create message
        sensor_msgs::msg::PointCloud2 message;

        // Read pcd from file
        pcl::PCDReader reader;
        pcl::PCLPointCloud2 cloud;
        reader.read(*sortedPCDsSet.begin(), cloud);
        sortedPCDsSet.erase(sortedPCDsSet.begin());

        emit progressChanged("Writing pcd file " + QString::number(iterationCount) + " of " + QString::number(frameCount) + "...",
                             ((float) iterationCount / (float) frameCount) * 100);
        iterationCount++;

        timeStamp += duration;
        // Write
        pcl_conversions::fromPCL(cloud, message);
        writer.write(message, m_topicName, timeStamp);
    }

    emit finished();
}
