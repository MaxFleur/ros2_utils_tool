#include "BagToPCDsThread.hpp"

#include "UtilsGeneral.hpp"
#include "UtilsROS.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include "rosbag2_cpp/reader.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

BagToPCDsThread::BagToPCDsThread(const Parameters::AdvancedParameters& parameters,
                                 unsigned int numberOfThreads, QObject* parent) :
    BasicThread(parameters.sourceDirectory, parameters.topicName, parent),
    m_parameters(parameters), m_numberOfThreads(numberOfThreads)
{
}


void
BagToPCDsThread::run()
{
    // Prepare parameters
    const auto messageCount = Utils::ROS::getTopicMessageCount(m_sourceDirectory, m_topicName);
    if (!messageCount) {
        emit failed();
        return;
    }

    const auto targetDirectoryStd = m_parameters.targetDirectory.toStdString();
    Utils::General::createAndClearDirectory(targetDirectoryStd);

    // Used to fill the file names with zeroes
    const auto messageCountNumberOfDigits = int(log10(*messageCount) + 1);

    rosbag2_cpp::Reader reader;
    reader.open(m_sourceDirectory);

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
    rosbag2_storage::SerializedBagMessageSharedPtr message;

    auto iterationCount = 0;
    std::mutex mutex;

    // Move to own lambda for multithreading
    const auto writePCD = [this, &targetDirectoryStd, &message, &iterationCount, &mutex, &reader,
                           serialization, messageCount, messageCountNumberOfDigits] {
        // Keep the message object local to each worker thread to avoid concurrent access to the same instance
        sensor_msgs::msg::PointCloud2 rosMessage;

        while (true) {
            mutex.lock();

            if (isInterruptionRequested() || !reader.has_next()) {
                mutex.unlock();
                break;
            }

            // Deserialize
            message = reader.read_next();
            if (message->topic_name != m_topicName) {
                mutex.unlock();
                continue;
            }
            rclcpp::SerializedMessage serializedMessage(*message->serialized_data);
            serialization.deserialize_message(&serializedMessage, &rosMessage);

            // Inform of progress update
            iterationCount++;
            emit progressChanged(QString("Writing PCD File %1 of %2...").arg(iterationCount).arg(*messageCount),
                                 (static_cast<float>(iterationCount) / static_cast<float>(*messageCount) * 100));

            // Have to create this as extra string to keep it atomic
            std::stringstream formattedIterationCount;
            // Use leading zeroes
            formattedIterationCount << std::setw(messageCountNumberOfDigits) << std::setfill('0') << iterationCount - 1;
            const auto targetString = targetDirectoryStd + "/" + formattedIterationCount.str() + ".pcd";

            mutex.unlock();

            // Convert to cloud and then write to pcd
            pcl::PCLPointCloud2 cloud;
            pcl_conversions::toPCL(rosMessage, cloud);
            pcl::PCDWriter writer;
            writer.write(targetString, cloud);
        }
    };

    while (reader.has_next()) {
        if (isInterruptionRequested()) {
            return;
        }
        // Multithread
        std::vector<std::thread> threadPool;
        for (unsigned int i = 0; i < m_numberOfThreads; ++i) {
            threadPool.emplace_back(writePCD);
        }
        for (auto& thread : threadPool) {
            thread.join();
        }
    }

    reader.close();
}
