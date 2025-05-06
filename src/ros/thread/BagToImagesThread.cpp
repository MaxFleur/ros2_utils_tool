#include "BagToImagesThread.hpp"

#include "UtilsROS.hpp"

#include <opencv2/imgcodecs.hpp>

#include "rosbag2_cpp/reader.hpp"

#include "sensor_msgs/msg/image.hpp"

#include <cmath>
#include <filesystem>

#ifdef ROS_HUMBLE
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

BagToImagesThread::BagToImagesThread(const Parameters::BagToImagesParameters& parameters,
                                     unsigned int numberOfThreads, QObject* parent) :
    BasicThread(parameters.sourceDirectory, parameters.topicName, parent),
    m_parameters(parameters), m_numberOfThreads(numberOfThreads)
{
}


void
BagToImagesThread::run()
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
    const auto messageCountNumberOfDigits = int(log10(*messageCount) + 1);

    rosbag2_cpp::Reader reader;
    reader.open(m_sourceDirectory);
    std::deque<rosbag2_storage::SerializedBagMessageSharedPtr> queue;
    constexpr int maximumInstancesForQueue = 100;

    rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
    auto iterationCount = 0;
    std::mutex mutex;

    // Read 100 messages into queue
    const auto readMessagesToQueue = [this, &reader, &queue] {
        auto i = 0;

        while (reader.has_next() && i < maximumInstancesForQueue) {
            if (isInterruptionRequested()) {
                return;
            }

            auto message = reader.read_next();
            if (message->topic_name != m_topicName) {
                continue;
            }
            queue.push_front(message);
            i++;
        }
    };

    cv_bridge::CvImagePtr cvPointer;

    const auto writeImageFromQueue = [this, &targetDirectoryStd, &mutex, &iterationCount, &queue, &cvPointer,
                                      serialization, messageCount, messageCountNumberOfDigits] {
        while (true) {
            mutex.lock();

            if (isInterruptionRequested() || queue.empty()) {
                mutex.unlock();
                break;
            }

            // Deserialize
            rclcpp::SerializedMessage serializedMessage(*queue.back()->serialized_data);
            auto rosMsg = std::make_shared<sensor_msgs::msg::Image>();
            serialization.deserialize_message(&serializedMessage, rosMsg.get());
            queue.pop_back();

            // Convert message to cv
            cvPointer = cv_bridge::toCvCopy(*rosMsg, rosMsg->encoding);
            // Convert to grayscale
            if (m_parameters.format == "png" && m_parameters.pngBilevel) {
                // Converting to a different channel seems to be saver then converting
                // to grayscale before calling imwrite
                cv::Mat mat(cvPointer->image.size(), CV_8UC1);
                mat.convertTo(cvPointer->image, CV_8UC1);
            } else if (m_parameters.useBWImages) {
                cv::cvtColor(cvPointer->image, cvPointer->image, cv::COLOR_BGR2GRAY);
            } else if (m_parameters.exchangeRedBlueValues) {
                cv::cvtColor(cvPointer->image, cvPointer->image, cv::COLOR_BGR2RGB);
            }

            // Inform of progress update
            iterationCount++;
            emit progressChanged("Writing image " + QString::number(iterationCount) + " of " + QString::number(*messageCount) + "...",
                                 ((float) iterationCount / (float) *messageCount) * 100);

            // Have to create this as extra string to keep it atomic inside the mutex
            std::stringstream formatedIterationCount;
            // Use leading zeroes
            formatedIterationCount << std::setw(messageCountNumberOfDigits) << std::setfill('0') << iterationCount;
            const auto targetString = targetDirectoryStd + "/" + formatedIterationCount.str() + "." + m_parameters.format.toStdString();

            mutex.unlock();
            // The main writing can be done in parallel
            cv::imwrite(targetString, cvPointer->image,
                        { m_parameters.format == "jpg" ? cv::IMWRITE_JPEG_QUALITY : cv::IMWRITE_PNG_COMPRESSION,
                          // Adjust the quality value to fit OpenCV param range
                          m_parameters.format == "jpg" ? (m_parameters.quality * 10) + 10 : m_parameters.quality,
                          m_parameters.format == "jpg" ? cv::IMWRITE_JPEG_OPTIMIZE : cv::IMWRITE_PNG_BILEVEL,
                          m_parameters.format == "jpg" ? m_parameters.jpgOptimize : m_parameters.pngBilevel });
        }
    };

    // Writing images might take lots of time, especially if higher compression is used. Thus, we aim to multithread the image writing.
    // However, the reader does not support parallel reading, thus we store the messages in a queue for parallel access.
    // Messages have a pretty large size and storing all of them at once might lead to an overflow quickly, though.
    // Thus, iterate through the bag in steps of 100 messages:
    // 1. Read 100 messages into the queue
    // 2. Write messages to images, emptying the queue
    // 3. Repeat until done
    while (reader.has_next()) {
        if (isInterruptionRequested()) {
            return;
        }

        readMessagesToQueue();

        std::vector<std::thread> threadPool;
        for (unsigned int i = 0; i < m_numberOfThreads; ++i) {
            threadPool.emplace_back(writeImageFromQueue);
        }
        for (auto& thread : threadPool) {
            thread.join();
        }
    }

    reader.close();

    emit finished();
}
