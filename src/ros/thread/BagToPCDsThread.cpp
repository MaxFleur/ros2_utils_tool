#include "BagToPCDsThread.hpp"

#include "UtilsROS.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include "rosbag2_cpp/reader.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <filesystem>

BagToPCDsThread::BagToPCDsThread(const Parameters::AdvancedParameters& parameters,
                                 unsigned int numberOfThreads, QObject* parent) :
    BasicThread(parameters.sourceDirectory, parameters.topicName, parent),
    m_parameters(parameters), m_numberOfThreads(numberOfThreads)
{
}


void
BagToPCDsThread::run()
{
    const auto targetDirectoryStd = m_parameters.targetDirectory.toStdString();

    if (!std::filesystem::exists(targetDirectoryStd)) {
        std::filesystem::create_directory(targetDirectoryStd);
    }
    if (!std::filesystem::is_empty(targetDirectoryStd)) {
        // Remove all images currently present
        for (const auto& entry : std::filesystem::directory_iterator(targetDirectoryStd)) {
            std::filesystem::remove_all(entry.path());
        }
    }

    // Prepare parameters
    const auto messageCount = Utils::ROS::getTopicMessageCount(m_sourceDirectory, m_topicName);
    const auto messageCountNumberOfDigits = int(log10(messageCount) + 1);

    rosbag2_cpp::Reader reader;
    reader.open(m_sourceDirectory);

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
    auto iterationCount = 0;
    std::mutex mutex;

    const auto writePCD = [this, &targetDirectoryStd, &reader, &mutex, &iterationCount,
                           serialization, messageCount, messageCountNumberOfDigits] {
        while (true) {
            mutex.lock();

            if (isInterruptionRequested() || !reader.has_next()) {
                mutex.unlock();
                break;
            }

            // Deserialize
            auto message = reader.read_next();
            if (message->topic_name != m_topicName) {
                mutex.unlock();
                continue;
            }
            rclcpp::SerializedMessage serializedMessage(*message->serialized_data);
            auto rosMsg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            serialization.deserialize_message(&serializedMessage, rosMsg.get());

            // Inform of progress update
            iterationCount++;
            emit progressChanged("Writing pcd file " + QString::number(iterationCount) + " of " + QString::number(messageCount) + "...",
                                 ((float) iterationCount / (float) messageCount) * 100);

            // Have to create this as extra string to keep it atomic inside the mutex
            std::stringstream formatedIterationCount;
            // Use leading zeroes
            formatedIterationCount << std::setw(messageCountNumberOfDigits) << std::setfill('0') << iterationCount;
            const auto targetString = targetDirectoryStd + "/" + formatedIterationCount.str() + ".pcd";

            mutex.unlock();

            // Convert to cloud and then write to pcd
            pcl::PCLPointCloud2 cloud;
            pcl_conversions::toPCL(*rosMsg, cloud);
            pcl::PCDWriter writer;
            writer.write(targetString, cloud);
        }
    };

    while (reader.has_next()) {
        if (isInterruptionRequested()) {
            return;
        }

        std::vector<std::thread> threadPool;
        for (unsigned int i = 0; i < m_numberOfThreads; ++i) {
            threadPool.emplace_back(writePCD);
        }
        for (auto& thread : threadPool) {
            thread.join();
        }
    }

    reader.close();

    emit finished();
}
